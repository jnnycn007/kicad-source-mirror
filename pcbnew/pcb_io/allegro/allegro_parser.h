/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2025 Jeff Wheeler <jeffwheeler@gmail.com>
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you may find one here:
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 * or you may search the http://www.gnu.org website for the version 2 license,
 * or you may write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 */

#ifndef ALLEGRO_PARSER_H_
#define ALLEGRO_PARSER_H_

#include <cstdarg>

#include <boost/version.hpp>
#if BOOST_VERSION >= 108100
#include <boost/unordered/unordered_flat_map.hpp>
#endif
#include <unordered_map>
#include <optional>

#include <wx/log.h>

#include <base_units.h>
#include <board.h>
#include <board_design_settings.h>
#include <footprint.h>
#include <geometry/eda_angle.h>
#include <layer_ids.h>
#include <lib_id.h>
#include <pad.h>
#include <pcb_shape.h>
#include <pcb_track.h>
#include <trigo.h>
#include <zone.h>

#include "allegro_file.h"
#include "allegro_structs.h"

#define PRINT_ALL_ITEMS 0

template <ALLEGRO::MAGIC magic>
class ALLEGRO_PARSER
{
public:
    ALLEGRO_PARSER( BOARD* aBoard, const ALLEGRO_FILE& aAllegroBrdFile );
    void Parse();

private:
    void BuildBoard();

    void ProcessLayers( const auto& aLayerSetList, const auto& aLayerNameFunc );
    void UpdateLayerInfo();

    void AddAnnotation( BOARD_ITEM_CONTAINER& aContainer, const ALLEGRO::T_14<magic>& aObjInst );
    void AddFootprint( const ALLEGRO::T_2B_FOOTPRINT<magic>& aBrdFootprint );

    void AddPad( FOOTPRINT* aFootprint, const ALLEGRO::T_32_PLACED_PAD<magic>& aBrdPlacedPad );

    void AddText( BOARD_ITEM_CONTAINER& aContainer, const ALLEGRO::T_30_STRING_GRAPHIC_WRAPPER<magic>& aStrWrapper );
    void AddTrack( const ALLEGRO::T_1B_NET<magic>& aBrdNet, const ALLEGRO::T_05_TRACK<magic>& aBrdTrack );
    void AddVia( const ALLEGRO::T_33_VIA<magic>& aBrdVia );
    void AddZone( BOARD_ITEM_CONTAINER& aContainer, const std::optional<ALLEGRO::T_1B_NET<magic>>& aBrdNet,
                  const ALLEGRO::T_28_SHAPE<magic>& aBrdShape );
    void AddRectangle( BOARD_ITEM_CONTAINER& aContainer, const ALLEGRO::T_24_RECTANGLE<magic>& aBrdRect );

    void             Skip( std::size_t aSkipBytes );
    void             Log( const char* fmt... );
    bool             IsType( uint32_t aKey, uint8_t aType );
    NETINFO_ITEM*    NetInfo( uint32_t aKey );
    NETINFO_ITEM*    NetInfo( const ALLEGRO::T_04_NET_ASSIGNMENT<magic>& aNetAssignment );
    SHAPE_LINE_CHAIN ShapeStartingAt( uint32_t* aKey );

    std::optional<wxString>     LookupString( uint32_t aKey ) const;
    std::optional<PCB_LAYER_ID> LookupPcbLayer( const ALLEGRO::LAYER_INFO& aLayer ) const;
    std::optional<wxString>     LookupBrdLayerName( const ALLEGRO::LAYER_INFO& aLayer ) const;

    template <class SEGMENT_TYPE>
    void AddSegmentToChain( SHAPE_LINE_CHAIN& aChain, SEGMENT_TYPE* aSegment, bool aFirstSegment );

    PCB_LAYER_ID EtchLayerToKi( const ALLEGRO::LAYER_INFO& aLayer );
    double       Scale( double aBrdValue );

    template <class T>
    std::pair<VECTOR2D, VECTOR2D> GetCoordinatePair( T& aInst );

    const ALLEGRO::T_36_08<magic>* FindFont( uint8_t aKey );

    constexpr uint32_t GetPadComponentCount( const ALLEGRO::T_1C_PAD_STACK<magic>& aPadStack );
    constexpr ALLEGRO::PAD_STACK_COMPONENT<magic>* GetPadComponent( const ALLEGRO::T_1C_PAD_STACK<magic>& aPadStack,
                                                                    uint32_t                              aIndex );
    void SetPadShape( PAD& aPad, const ALLEGRO::PAD_STACK_COMPONENT<magic>& aPadStackComponent );

    std::optional<wxString> RefdesLookup( const ALLEGRO::T_2D_PLACED_FOOTPRINT<magic>& aPlacedFootprint );

    template <template <ALLEGRO::MAGIC> typename T>
    static uint32_t DefaultParser( ALLEGRO_PARSER& aParser );

    static uint32_t Parse03( ALLEGRO_PARSER& aParser );
    static uint32_t Parse1CPadStack( ALLEGRO_PARSER& aParser );
    static uint32_t Parse1D( ALLEGRO_PARSER& aParser );
    static uint32_t Parse1EModelInfo( ALLEGRO_PARSER& aParser );
    static uint32_t Parse1F( ALLEGRO_PARSER& aParser );
    static uint32_t Parse21Metadata( ALLEGRO_PARSER& aParser );
    static uint32_t Parse27( ALLEGRO_PARSER& aParser );
    static uint32_t Parse2ALayerInfo( ALLEGRO_PARSER& aParser );
    static uint32_t Parse31StringGraphic( ALLEGRO_PARSER& aParser );
    static uint32_t Parse35( ALLEGRO_PARSER& aParser );
    static uint32_t Parse36( ALLEGRO_PARSER& aParser );
    static uint32_t Parse3B( ALLEGRO_PARSER& aParser );
    static uint32_t Parse3C( ALLEGRO_PARSER& aParser );

    typedef uint32_t ( *PARSER_FUNC )( ALLEGRO_PARSER& aParser );

    static constexpr PARSER_FUNC PARSER_TABLE[] = {
        // 0x00
        nullptr,
        // 0x01
        &DefaultParser<ALLEGRO::T_01_ARC>,
        // 0x02
        nullptr,
        // 0x03
        &Parse03,
        // 0x04
        &DefaultParser<ALLEGRO::T_04_NET_ASSIGNMENT>,
        // 0x05
        &DefaultParser<ALLEGRO::T_05_TRACK>,
        // 0x06
        &DefaultParser<ALLEGRO::T_06>,
        // 0x07
        &DefaultParser<ALLEGRO::T_07>,
        // 0x08
        &DefaultParser<ALLEGRO::T_08>,
        // 0x09
        &DefaultParser<ALLEGRO::T_09>,
        // 0x0A
        &DefaultParser<ALLEGRO::T_0A>,
        // 0x0B
        nullptr,
        // 0x0C
        &DefaultParser<ALLEGRO::T_0C>,
        // 0x0D
        &DefaultParser<ALLEGRO::T_0D_PAD>,
        // 0x0E
        &DefaultParser<ALLEGRO::T_0E>,
        // 0x0F
        &DefaultParser<ALLEGRO::T_0F>,
        // 0x10
        &DefaultParser<ALLEGRO::T_10>,
        // 0x11
        &DefaultParser<ALLEGRO::T_11>,
        // 0x12
        &DefaultParser<ALLEGRO::T_12>,
        // 0x13
        nullptr,
        // 0x14
        &DefaultParser<ALLEGRO::T_14>,
        // 0x15
        &DefaultParser<ALLEGRO::T_15_SEGMENT>,
        // 0x16
        &DefaultParser<ALLEGRO::T_16_SEGMENT>,
        // 0x17
        &DefaultParser<ALLEGRO::T_17_SEGMENT>,
        // 0x18
        nullptr,
        // 0x19
        nullptr,
        // 0x1A
        nullptr,
        // 0x1B
        &DefaultParser<ALLEGRO::T_1B_NET>,
        // 0x1C
        &Parse1CPadStack,
        // 0x1D
        &Parse1D,
        // 0x1E
        &Parse1EModelInfo,
        // 0x1F
        &Parse1F,
        // 0x20
        &DefaultParser<ALLEGRO::T_20>,
        // 0x21
        &Parse21Metadata,
        // 0x22
        &DefaultParser<ALLEGRO::T_22>,
        // 0x23
        &DefaultParser<ALLEGRO::T_23_RAT>,
        // 0x24
        &DefaultParser<ALLEGRO::T_24_RECTANGLE>,
        // 0x25
        nullptr,
        // 0x26
        &DefaultParser<ALLEGRO::T_26>,
        // 0x27
        &Parse27,
        // 0x28
        &DefaultParser<ALLEGRO::T_28_SHAPE>,
        // 0x29
        nullptr,
        // 0x2A
        &Parse2ALayerInfo,
        // 0x2B
        &DefaultParser<ALLEGRO::T_2B_FOOTPRINT>,
        // 0x2C
        &DefaultParser<ALLEGRO::T_2C_TABLE>,
        // 0x2D
        &DefaultParser<ALLEGRO::T_2D_PLACED_FOOTPRINT>,
        // 0x2E
        &DefaultParser<ALLEGRO::T_2E>,
        // 0x2F
        &DefaultParser<ALLEGRO::T_2F>,
        // 0x30
        &DefaultParser<ALLEGRO::T_30_STRING_GRAPHIC_WRAPPER>,
        // 0x31
        &Parse31StringGraphic,
        // 0x32
        &DefaultParser<ALLEGRO::T_32_PLACED_PAD>,
        // 0x33
        &DefaultParser<ALLEGRO::T_33_VIA>,
        // 0x34
        &DefaultParser<ALLEGRO::T_34_RULE_REGION>,
        // 0x35
        &Parse35,
        // 0x36
        &Parse36,
        // 0x37
        &DefaultParser<ALLEGRO::T_37>,
        // 0x38
        &DefaultParser<ALLEGRO::T_38_FILM>,
        // 0x39
        &DefaultParser<ALLEGRO::T_39_FILM_LAYER_LIST>,
        // 0x3A
        &DefaultParser<ALLEGRO::T_3A_FILM_LAYER_LIST_NODE>,
        // 0x3B
        &Parse3B,
        // 0x3C
        &Parse3C,
    };

    BOARD*              m_board;
    const ALLEGRO_FILE& m_brd;

    ALLEGRO::HEADER* m_header = nullptr;
    void*            m_baseAddr = nullptr;
    void*            m_curAddr = nullptr;

    double m_scaleFactor = 0;

    // 0 = not calculated yet
    uint16_t m_layerCount = 0;

    std::map<uint32_t, char*>                          m_strings;
    std::map<uint32_t, ALLEGRO::T_2A_LAYER_SET<magic>> m_layerSetMap;
    std::map<uint32_t, ALLEGRO::T_1E>                  m_t1E_map;
    std::map<uint32_t, ALLEGRO::T_36<magic>*>          m_t36_map;

#if BOOST_VERSION >= 108100
    boost::unordered_flat_map<uint32_t, void*> m_ptrs;
#else
    std::unordered_map<uint32_t, void*> m_ptrs;
#endif
};

// Reimplemented so we do not require all of the corresponding header.
constexpr uint32_t local_ntohl( uint32_t aX )
{
    return ( ( aX & 0x000000FF ) << 24 ) | ( ( aX & 0x0000FF00 ) << 8 ) | ( ( aX & 0x00FF0000 ) >> 8 )
           | ( ( aX & 0xFF000000 ) >> 24 );
}

// Helper function for building parser map below
template <ALLEGRO::MAGIC magic>
const std::function<void( BOARD*, const ALLEGRO_FILE& )> ParserFunc()
{
    return []( BOARD* aBoard, const ALLEGRO_FILE& aAllegroBrdFile )
    {
        ALLEGRO_PARSER<magic>( aBoard, aAllegroBrdFile ).Parse();
    };
}

const std::unordered_map<uint32_t, std::function<void( BOARD*, const ALLEGRO_FILE& )>> PARSERS = { {
        { 0x00130000, ParserFunc<ALLEGRO::A_160>() }, { 0x00130200, ParserFunc<ALLEGRO::A_160>() },
        { 0x00130402, ParserFunc<ALLEGRO::A_162>() }, { 0x00130C03, ParserFunc<ALLEGRO::A_164>() },
        { 0x00131003, ParserFunc<ALLEGRO::A_165>() }, { 0x00131503, ParserFunc<ALLEGRO::A_166>() },
        { 0x00131504, ParserFunc<ALLEGRO::A_166>() }, { 0x00140400, ParserFunc<ALLEGRO::A_172>() },
        { 0x00140500, ParserFunc<ALLEGRO::A_172>() }, { 0x00140501, ParserFunc<ALLEGRO::A_172>() },
        { 0x00140502, ParserFunc<ALLEGRO::A_172>() }, { 0x00140600, ParserFunc<ALLEGRO::A_172>() },
        { 0x00140700, ParserFunc<ALLEGRO::A_172>() }, { 0x00140900, ParserFunc<ALLEGRO::A_174>() },
        { 0x00140901, ParserFunc<ALLEGRO::A_174>() }, { 0x00140902, ParserFunc<ALLEGRO::A_174>() },
        { 0x00140E00, ParserFunc<ALLEGRO::A_174>() }, { 0x00141500, ParserFunc<ALLEGRO::A_175>() },
        { 0x00141501, ParserFunc<ALLEGRO::A_175>() }, { 0x00141502, ParserFunc<ALLEGRO::A_175>() },
} };

#endif // ALLEGRO_PARSER_H_
