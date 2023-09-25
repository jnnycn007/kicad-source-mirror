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

#include "allegro_parser.h"

#include <wx/string.h>

// Helper functions
uint32_t round_to_word( uint32_t aLength )
{
    if( aLength % 4 != 0 )
    {
        return aLength / 4 * 4 + 4;
    }
    else
    {
        return aLength;
    }
}

double cfp_to_double( ALLEGRO::CADENCE_FP aAllegroFloat )
{
    ALLEGRO::CADENCE_FP swapped{
        .x = aAllegroFloat.y,
        .y = aAllegroFloat.x,
    };
    double g = 0.;
    std::memcpy( &g, &swapped, 8 );
    return g;
}

template <ALLEGRO::MAGIC magic>
ALLEGRO_PARSER<magic>::ALLEGRO_PARSER( BOARD* aBoard, const ALLEGRO_FILE& aAllegroBrdFile ) :
        m_board( aBoard ), m_brd( aAllegroBrdFile ){};

template <ALLEGRO::MAGIC magic>
void ALLEGRO_PARSER<magic>::Parse()
{
    m_baseAddr = m_brd.region.get_address();
    size_t size = m_brd.region.get_size();

    m_curAddr = m_baseAddr;

    m_header = static_cast<ALLEGRO::HEADER*>( m_curAddr );
    Skip( sizeof( ALLEGRO::HEADER ) );

    m_ptrs.reserve( m_header->object_count );

    // Cache a fixed scale factor
    switch( m_header->units )
    {
    case ALLEGRO::BRD_UNITS::IMPERIAL: m_scaleFactor = ( (double) 25400. ) / m_header->unit_divisor; break;
    case ALLEGRO::BRD_UNITS::METRIC: m_scaleFactor = ( (double) 1000000. ) / m_header->unit_divisor; break;
    default: THROW_IO_ERROR( wxString::Format( "Units 0x%02hhX not recognized.", m_header->units ) );
    };

    // Strings map
    m_curAddr = (char*) m_baseAddr + 0x1200;
    for( uint32_t i = 0; i < m_header->strings_count; i++ )
    {
        uint32_t id = *( (uint32_t*) ( m_curAddr ) );
        Skip( sizeof( id ) );

        m_strings[id] = (char*) m_curAddr;

        // Add one to include the NULL byte that might force the length to one
        // word longer.
        uint32_t len = strlen( (char*) m_curAddr );
        Skip( round_to_word( len + 1 ) );

        // wxLogMessage( "%08X = %s\n", id, m_strings[id] );
    }


    // All other objects
    while( m_curAddr < (char*) m_baseAddr + size && *static_cast<uint8_t*>( m_curAddr ) != 0x00 )
    {
        uint8_t t = *static_cast<uint8_t*>( m_curAddr );
        if( PRINT_ALL_ITEMS )
        {
            Log( "Handling t=0x%02X: ", t );
        }

        const PARSER_FUNC parser = PARSER_TABLE[t];
        if( t < sizeof( PARSER_TABLE ) / sizeof( PARSER_FUNC ) && parser != nullptr )
        {
            parser( *this );
        }
        else
        {
            THROW_IO_ERROR( wxString::Format( "Do not have parser for t=0x%02X available.", local_ntohl( t ) ) );
        }
    }

    BuildBoard();
}

template <ALLEGRO::MAGIC magic>
template <template <ALLEGRO::MAGIC> typename T>
uint32_t ALLEGRO_PARSER<magic>::DefaultParser( ALLEGRO_PARSER<magic>& aParser )
{
    auto* inst = static_cast<T<magic>*>( aParser.m_curAddr );
    aParser.m_ptrs[inst->k] = aParser.m_curAddr;
    aParser.Skip( ALLEGRO::sizeof_allegro_obj<T<magic>>() );
    return inst->k;
};

template <ALLEGRO::MAGIC magic>
uint32_t ALLEGRO_PARSER<magic>::Parse03( ALLEGRO_PARSER<magic>& aParser )
{
    auto*    i = static_cast<ALLEGRO::T_03<magic>*>( aParser.m_curAddr );
    uint32_t k = ALLEGRO_PARSER<magic>::DefaultParser<ALLEGRO::T_03>( aParser );

    char* buf;
    // inst.has_str = false;
    uint32_t size;

    // log(base_addr_glb, address, "- Subtype.t = 0x%02X\n", i->subtype.t);
    switch( i->subtype.t & 0xFF )
    {
    case 0x65: break;
    case 0x64:
    case 0x66:
    case 0x67:
    case 0x6A: aParser.Skip( 4 ); break;
    case 0x6D:
    case 0x6E:
    case 0x6F:
    case 0x68:
    case 0x6B:
    case 0x71:
    case 0x73:
    case 0x78: aParser.Skip( round_to_word( i->subtype.size ) ); break;
    case 0x69: aParser.Skip( 8 ); break;
    case 0x6C:
        size = *( static_cast<uint32_t*>( (void*) aParser.m_curAddr ) );
        aParser.Skip( 4 + 4 * size );
        break;
    case 0x70:
    case 0x74:
        uint16_t x[2];
        x[0] = *( static_cast<uint16_t*>( (void*) aParser.m_curAddr ) );
        aParser.Skip( 2 );
        x[1] = *( static_cast<uint16_t*>( (void*) aParser.m_curAddr ) );
        aParser.Skip( 2 );
        aParser.Skip( x[1] + 4 * x[0] );
        break;
    case 0xF6: aParser.Skip( 80 ); break;
    default: THROW_IO_ERROR( wxString::Format( "Unexpected value subtype t=0x%02X", i->subtype.t ) );
    };

    return 0;
};

template <ALLEGRO::MAGIC magic>
uint32_t ALLEGRO_PARSER<magic>::Parse1CPadStack( ALLEGRO_PARSER<magic>& aParser )
{
    auto*    i = static_cast<ALLEGRO::T_1C_PAD_STACK<magic>*>( aParser.m_curAddr );
    uint32_t k = ALLEGRO_PARSER<magic>::DefaultParser<ALLEGRO::T_1C_PAD_STACK>( aParser );

    uint16_t size = 0;
    if constexpr( magic < ALLEGRO::A_172 )
    {
        // printf("layer count 1 %d\n", i->layer_count);
        size = 10 + i->layer_count * 3;
    }
    else
    {
        // printf("layer count 2 %d\n", i->layer_count);
        size = 21 + i->layer_count * 4;
    }

    for( uint32_t i = 0; i < size; i++ )
    {
        // log(fs.region.get_address(), address, "- Skipping t13 %d\n", i);
        // log_n_words(address, 4);
        aParser.Skip( ALLEGRO::sizeof_allegro_obj<ALLEGRO::PAD_STACK_COMPONENT<magic>>() );
    }
    // skip(address, size * sizeof_until_tail<t13<magic>>());

    if constexpr( magic >= ALLEGRO::A_172 )
    {
        aParser.Skip( i->n * 40 );
    }
    else
    {
        aParser.Skip( i->n * 32 - 4 );
    }

    return k;
};

template <ALLEGRO::MAGIC magic>
uint32_t ALLEGRO_PARSER<magic>::Parse1D( ALLEGRO_PARSER<magic>& aParser )
{
    auto*    i = static_cast<ALLEGRO::T_1D<magic>*>( aParser.m_curAddr );
    uint32_t k = ALLEGRO_PARSER<magic>::DefaultParser<ALLEGRO::T_1D>( aParser );

    // log(&f, "size_a = %d, size_b = %d\n", size_a, size_b);
    aParser.Skip( i->size_b * ( magic >= ALLEGRO::A_162 ? 56 : 48 ) );
    aParser.Skip( i->size_a * 256 );
    if( magic >= ALLEGRO::A_172 )
    {
        aParser.Skip( 4 );
    }
    return 0;
};

template <ALLEGRO::MAGIC magic>
uint32_t ALLEGRO_PARSER<magic>::Parse1EModelInfo( ALLEGRO_PARSER<magic>& aParser )
{
    ALLEGRO::T_1E* x1E_inst = new ALLEGRO::T_1E;
    memcpy( x1E_inst, aParser.m_curAddr, sizeof( ALLEGRO::T_1E_HEADER ) );
    aParser.Skip( sizeof( ALLEGRO::T_1E_HEADER ) );
    // f.read((char*)x1E_inst, sizeof(x1E_hdr));
    x1E_inst->s = (char*) aParser.m_curAddr;
    aParser.Skip( round_to_word( x1E_inst->hdr.size ) );
    // f.read(x1E_inst->s, round_to_word(x1E_inst->hdr.size));
    aParser.m_t1E_map[x1E_inst->hdr.k] = *x1E_inst;
    if( magic >= ALLEGRO::A_172 )
    {
        aParser.Skip( 4 );
    }
    return 0;
}

template <ALLEGRO::MAGIC magic>
uint32_t ALLEGRO_PARSER<magic>::Parse1F( ALLEGRO_PARSER<magic>& aParser )
{
    auto*    i = static_cast<ALLEGRO::T_1F<magic>*>( aParser.m_curAddr );
    uint32_t k = ALLEGRO_PARSER<magic>::DefaultParser<ALLEGRO::T_1F>( aParser );

    if constexpr( magic >= ALLEGRO::A_175 )
    {
        aParser.Skip( i->size * 384 + 8 );
    }
    else if constexpr( magic >= ALLEGRO::A_172 )
    {
        aParser.Skip( i->size * 280 + 8 );
    }
    else if constexpr( magic >= ALLEGRO::A_162 )
    {
        aParser.Skip( i->size * 280 + 4 );
    }
    else
    {
        aParser.Skip( i->size * 240 + 4 );
    }

    return k;
}

template <ALLEGRO::MAGIC magic>
uint32_t ALLEGRO_PARSER<magic>::Parse21Metadata( ALLEGRO_PARSER& aParser )
{
    auto* i = static_cast<ALLEGRO::T_21_HEADER*>( aParser.m_curAddr );
    if( i->r == 1304 )
    {
        ALLEGRO::STACKUP_MATERIAL ps;
        memcpy( &ps, aParser.m_curAddr, sizeof( ALLEGRO::STACKUP_MATERIAL ) );
        aParser.Skip( sizeof( ALLEGRO::STACKUP_MATERIAL ) );
        // fs.stackup_materials[ps.hdr.k] = ps;
        // log(&f, "- - Stackup material... %02d->%s\n", ps.layer_id,
        // ps.material);
    }
    else if( i->r == 14093 )
    {
        // log(&f, "- - i->r=\x1b[31m14093\x1b[0m...?\n");
        aParser.Skip( i->size );
        // } else if (i->r == 2050) {
    }
    else if( i->r == 0x0407 )
    {
        ALLEGRO::META_NETLIST_PATH r;
        aParser.Skip( ALLEGRO::sizeof_allegro_obj<ALLEGRO::META_NETLIST_PATH>() );
        aParser.Skip( 1028 );
        // char s[1028];
        // f.read((char*)&r, sizeof_until_tail<meta_netlist_path>());
        // f.read(s, sizeof(s));
        // r.path = std::string(s);
        // fs.netlist_path = r;
    }
    else
    {
        aParser.Skip( i->size );
    }
    return 0;
};

template <ALLEGRO::MAGIC magic>
uint32_t ALLEGRO_PARSER<magic>::Parse27( ALLEGRO_PARSER& aParser )
{
    aParser.m_curAddr = (char*) aParser.m_baseAddr + aParser.m_header->x27_end_offset - 1;
    return 0;
}

template <ALLEGRO::MAGIC magic>
uint32_t ALLEGRO_PARSER<magic>::Parse2ALayerInfo( ALLEGRO_PARSER& aParser )
{
    ALLEGRO::T_2A_LAYER_SET<magic> layer_set;
    memcpy( &layer_set.hdr, aParser.m_curAddr, sizeof( ALLEGRO::T_2A_HEADER ) );
    aParser.Skip( sizeof( ALLEGRO::T_2A_HEADER ) );

    if( magic >= ALLEGRO::A_174 )
    {
        aParser.Skip( 4 );
    }

    for( uint16_t i = 0; i < layer_set.hdr.count; i++ )
    {
        if constexpr( magic <= ALLEGRO::A_164 )
        {
            auto entry = static_cast<ALLEGRO::T_2A_LOCAL_LAYER_PROPERTIES*>( aParser.m_curAddr );
            aParser.Skip( sizeof( *entry ) );
            layer_set.local_entries.push_back( *entry );
        }
        else
        {
            auto entry = static_cast<ALLEGRO::T_2A_REFERENCE_LAYER_PROPERTIES*>( aParser.m_curAddr );
            aParser.Skip( sizeof( *entry ) );
            layer_set.reference_entries.push_back( *entry );
        }
    }
    layer_set.k = *static_cast<uint32_t*>( aParser.m_curAddr );
    aParser.Skip( 4 );
    aParser.m_layerSetMap[layer_set.k] = layer_set;
    return 0;
}

template <ALLEGRO::MAGIC magic>
uint32_t ALLEGRO_PARSER<magic>::Parse31StringGraphic( ALLEGRO_PARSER& aParser )
{
    auto*    i = static_cast<ALLEGRO::T_31_STRING_GRAPHIC<magic>*>( aParser.m_curAddr );
    uint32_t k = ALLEGRO_PARSER<magic>::DefaultParser<ALLEGRO::T_31_STRING_GRAPHIC>( aParser );

    if( i->len > 0 )
    {
        uint32_t len = round_to_word( i->len );
        aParser.Skip( len );
    }

    return 0;
}

template <ALLEGRO::MAGIC magic>
uint32_t ALLEGRO_PARSER<magic>::Parse35( ALLEGRO_PARSER& aParser )
{
    aParser.Skip( 124 );
    return 0;
}

template <ALLEGRO::MAGIC magic>
uint32_t ALLEGRO_PARSER<magic>::Parse36( ALLEGRO_PARSER& aParser )
{
    auto* inst = static_cast<ALLEGRO::T_36<magic>*>( aParser.m_curAddr );
    // memcpy( &inst, parser.m_curAddr, ALLEGRO::sizeof_allegro_obj<ALLEGRO::T_36<magic>>() );
    aParser.Skip( ALLEGRO::sizeof_allegro_obj<ALLEGRO::T_36<magic>>() );
    // f.read((char*)inst, sizeof_until_tail<x36<version>>());

    // log(&f, "- x36\n");
    // log(&f, "- - inst->c    = 0x %04X\n", ntohs(inst->c));
    // log(&f, "- - inst->k    = 0x %08X\n", local_ntohl(inst->k));
    // log(&f, "- - inst->next = 0x %08X\n", local_ntohl(inst->next));
    // if constexpr (!std::is_same_v<decltype(inst->un1), std::monostate>) {
    //     log(&f, "- - inst->un1  = 0x %08X\n", local_ntohl(inst->un1));
    // } else {
    //     log(&f, "- - inst->un1  = n/a\n");
    // }
    // log(&f, "- - inst->size = %d\n", inst->size);
    // log(&f,
    //     "- - inst->count = %d, inst->last_idx = %d, inst->un3 = 0x"
    //     " % 08X\n ",
    //     inst->count, inst->last_idx, local_ntohl(inst->un3));
    // if constexpr (!std::is_same_v<decltype(inst->un2), std::monostate>) {
    //     log(&f, "- - inst->un2  = 0x %08X\n", local_ntohl(inst->un2));
    // } else {
    //     log(&f, "- - inst->un2  = n/a\n");
    // }

    switch( inst->c )
    {
    case 0x02:
        for( uint32_t i = 0; i < inst->size; i++ )
        {
            // Discard
            auto inst_x02 = *static_cast<ALLEGRO::x36_x02<magic>*>( aParser.m_curAddr );
            aParser.Skip( ALLEGRO::sizeof_allegro_obj<ALLEGRO::x36_x02<magic>>() );
            // f.read((char*)&inst_x02,
            // sizeof_until_tail<x36_x02<version>>()); log(&f, "- x02.str =
            // %s\n", inst_x02.str);
        }
        break;
    case 0x06:
        for( uint32_t i = 0; i < inst->size; i++ )
        {
            // Currently just read the object and immediately throw it
            // away, because we don't know of any use for it.
            auto inst_x06 = *static_cast<ALLEGRO::x36_x06<magic>*>( aParser.m_curAddr );
            aParser.Skip( ALLEGRO::sizeof_allegro_obj<ALLEGRO::x36_x06<magic>>() );
            // f.read((char*)&inst_x06,
            // sizeof_until_tail<x36_x06<version>>()); log(&f, "- x06.n =
            // %d\n", inst_x06.n); log(&f, "- x06.r = %d\n", inst_x06.r);
            // log(&f, "- x06.s = %d\n", inst_x06.s);
            // log(&f, "- x06.un1 = %d\n\n", inst_x06.un1);
        }
        break;
    case 0x03:
        for( uint32_t i = 0; i < inst->size; i++ )
        {
            // Just throw this away after reading it.
            auto inst_x03 = *static_cast<ALLEGRO::x36_x03<magic>*>( aParser.m_curAddr );
            aParser.Skip( ALLEGRO::sizeof_allegro_obj<ALLEGRO::x36_x03<magic>>() );
            // f.read((char*)&inst_x03,
            // sizeof_until_tail<x36_x03<version>>()); if (version >= A_172)
            // {
            //     log(&f, "- - s = \"%s\"\n", inst_x03.str);
            // } else {
            //     log(&f, "- - s = \"%s\"\n", inst_x03.str_16x);
            // }
        }
        break;
    case 0x05: aParser.Skip( inst->size * 28 ); break;
    case 0x08:
        for( uint32_t i = 0; i < inst->size; i++ )
        {
            auto x = *static_cast<ALLEGRO::T_36_08<magic>*>( aParser.m_curAddr );
            aParser.Skip( ALLEGRO::sizeof_allegro_obj<ALLEGRO::T_36_08<magic>>() );
            // f.read((char*)&x, sizeof_until_tail<T_36_08<version>>());
            // inst->x08s.push_back( x );
            // log(&f,
            //     "- - 0x%02X %08x %08x"
            //     " xs={h=% 7d w=% 7d % 7d % 7d % 7d %7d}",
            //     i, local_ntohl(x.a), local_ntohl(x.b), x.char_height, x.char_width,
            //     x.xs[0], x.xs[1], x.xs[2], x.xs[3]);
            // if constexpr (!std::is_same_v<decltype(x.ys),
            // std::monostate>) {
            //     printf(" ys={% 7d % 7d % 7d}", x.ys[0], x.ys[1],
            //     x.ys[2]);
            // }
            // printf("\n");
        }
        break;
    case 0x0B:
        // No clue what this represents?
        aParser.Skip( inst->size * 1016 );
        // for (uint32_t i = 0; i < inst->size; i++) {
        //     log(&f, "- - Next words:");
        //     log_n_words(&f, 6);
        //     skip(&f, 1016 - 4 * 6);
        // }
        break;
    case 0x0C: aParser.Skip( inst->size * 232 ); break;
    case 0x0D: aParser.Skip( inst->size * 200 ); break;
    case 0x0F:
        for( uint32_t i = 0; i < inst->size; i++ )
        {
            auto x = *static_cast<ALLEGRO::x36_x0F<magic>*>( aParser.m_curAddr );
            aParser.Skip( sizeof( ALLEGRO::x36_x0F<magic> ) );
            // f.read((char*)&x, sizeof(x36_x0F<version>));
            // inst->x0Fs.push_back( x );
        }
        break;
    case 0x10:
        aParser.Skip( inst->size * 108 );
        // for (uint32_t i = 0; i < inst->size; i++) {
        //     log(&f, "- - Next words:");
        //     log_n_words(&f, 6);
        //     skip(&f, 108 - 4 * 6);
        // }
        break;
    default: THROW_IO_ERROR( wxString::Format( "Do not know how to handle c=%X", inst->c ) );
    }

    aParser.m_t36_map[inst->k] = inst;
    // parser.m_t36_map.insert( &inst );
    // fs.x36_map[inst->k] = upgrade<version, A_174>( inst );
    return 0;
}

template <ALLEGRO::MAGIC magic>
uint32_t ALLEGRO_PARSER<magic>::Parse3B( ALLEGRO_PARSER& aParser )
{
    ALLEGRO::T_3B<magic>* i = (ALLEGRO::T_3B<magic>*) aParser.m_curAddr;

    aParser.Skip( ALLEGRO::sizeof_allegro_obj<ALLEGRO::T_3B<magic>>() );
    aParser.Skip( round_to_word( i->len ) );

    return 0;
}

template <ALLEGRO::MAGIC magic>
uint32_t ALLEGRO_PARSER<magic>::Parse3C( ALLEGRO_PARSER& aParser )
{
    auto*    i = static_cast<ALLEGRO::T_3C<magic>*>( aParser.m_curAddr );
    uint32_t k = ALLEGRO_PARSER<magic>::DefaultParser<ALLEGRO::T_3C>( aParser );

    aParser.Skip( i->size * 4 );

    return 0;
}

template <ALLEGRO::MAGIC magic>
void ALLEGRO_PARSER<magic>::ProcessLayers( const auto& aLayerSetEntries, const auto& aLayerNameFunc )
{
    m_layerCount = aLayerSetEntries.size();
    m_board->SetCopperLayerCount( m_layerCount );

    uint8_t i = 0;
    for( const auto& entry : aLayerSetEntries )
    {
        auto layer = ALLEGRO::LAYER_INFO{ .family = ALLEGRO::LAYER_FAMILY::COPPER, .ordinal = i++ };
        m_board->SetLayerName( EtchLayerToKi( layer ), *LookupBrdLayerName( layer ) );
    }
}

template <ALLEGRO::MAGIC magic>
void ALLEGRO_PARSER<magic>::UpdateLayerInfo()
{
    auto     layer_set = m_header->layer_sets.at( static_cast<uint8_t>( ALLEGRO::LAYER_FAMILY::COPPER ) );
    uint32_t ptr = layer_set.layer_set_ptr;

    if( m_layerSetMap.count( ptr ) > 0 )
    {
        const ALLEGRO::T_2A_LAYER_SET<magic>* x = &m_layerSetMap.at( ptr );
        if constexpr( magic <= ALLEGRO::A_164 )
        {
            ProcessLayers( x->local_entries,
                           []( const auto& entry )
                           {
                               return entry.layer_name;
                           } );
        }
        else
        {
            ProcessLayers( x->reference_entries,
                           [this]( const auto& entry )
                           {
                               return *LookupString( entry.layer_name_str_ptr );
                           } );
        }
    }
}

template <ALLEGRO::MAGIC magic>
void ALLEGRO_PARSER<magic>::AddAnnotation( BOARD_ITEM_CONTAINER& aContainer, const ALLEGRO::T_14<magic>& aObjInst )
{
    PCB_LAYER_ID layer = LookupPcbLayer( aObjInst.layer ).value_or( User_1 );

    uint32_t k = aObjInst.ptr2;
    while( true )
    {
        if( IsType( k, 0x01 ) )
        {
            auto* i01 = static_cast<ALLEGRO::T_01_ARC<magic>*>( m_ptrs[k] );

            double r = cfp_to_double( i01->r );

            auto [start, end] = GetCoordinatePair( *i01 );
            VECTOR2D center{ Scale( (int32_t) cfp_to_double( i01->x ) ), Scale( -(int32_t) cfp_to_double( i01->y ) ) };
            VECTOR2I mid = CalcArcMid( start, end, center );

            PCB_SHAPE* arc = new PCB_SHAPE( &aContainer, SHAPE_T::ARC );

            arc->SetLayer( layer );
            arc->SetWidth( Scale( i01->width ) );
            arc->SetArcGeometry( start, mid, end );

            aContainer.Add( arc, ADD_MODE::APPEND );

            k = i01->next;
        }
        else if( IsType( k, 0x15 ) )
        {
            auto* i15 = static_cast<ALLEGRO::T_15_SEGMENT<magic>*>( m_ptrs[k] );
            auto [start, end] = GetCoordinatePair( *i15 );
            PCB_SHAPE* segment = new PCB_SHAPE( &aContainer, SHAPE_T::SEGMENT );

            segment->SetLayer( layer );
            segment->SetWidth( Scale( i15->width ) );
            segment->SetStart( start );
            segment->SetEnd( end );

            aContainer.Add( segment, ADD_MODE::APPEND );

            k = i15->next;
        }
        else if( IsType( k, 0x16 ) )
        {
            auto* i16 = static_cast<ALLEGRO::T_16_SEGMENT<magic>*>( m_ptrs[k] );
            auto [start, end] = GetCoordinatePair( *i16 );
            PCB_SHAPE* segment = new PCB_SHAPE( &aContainer, SHAPE_T::SEGMENT );

            segment->SetLayer( layer );
            segment->SetWidth( Scale( i16->width ) );
            segment->SetStart( start );
            segment->SetEnd( end );

            aContainer.Add( segment, ADD_MODE::APPEND );

            k = i16->next;
        }
        else if( IsType( k, 0x17 ) )
        {
            auto* i17 = static_cast<ALLEGRO::T_17_SEGMENT<magic>*>( m_ptrs[k] );
            auto [start, end] = GetCoordinatePair( *i17 );
            PCB_SHAPE* segment = new PCB_SHAPE( &aContainer, SHAPE_T::SEGMENT );

            segment->SetLayer( layer );
            segment->SetWidth( Scale( i17->width ) );
            segment->SetStart( start );
            segment->SetEnd( end );

            aContainer.Add( segment, ADD_MODE::APPEND );

            k = i17->next;
        }
        else
        {
            break;
        }
    }
}

template <ALLEGRO::MAGIC magic>
void ALLEGRO_PARSER<magic>::AddFootprint( const ALLEGRO::T_2B_FOOTPRINT<magic>& aBrdFootprint )
{
    // wxLogMessage( "Adding footprint" );
    wxASSERT_MSG( m_strings.count( aBrdFootprint.footprint_string_ref ) == 1, "Expected a string" );
    wxString footprint_name = m_strings[aBrdFootprint.footprint_string_ref];
    LIB_ID   lib_id = LIB_ID( "my_lib", footprint_name );

    uint32_t k = aBrdFootprint.ptr2;
    while( IsType( k, 0x2D ) )
    {
        auto* i2D = static_cast<ALLEGRO::T_2D_PLACED_FOOTPRINT<magic>*>( m_ptrs[k] );

        std::unique_ptr<FOOTPRINT> fp = std::make_unique<FOOTPRINT>( m_board );
        fp->SetAttributes( FOOTPRINT_ATTR_T::FP_SMD );
        fp->SetFPID( lib_id );

        // Refdes
        std::optional<wxString> refdes = RefdesLookup( *i2D );
        if( refdes )
        {
            fp->SetReference( *refdes );
        }
        else
        {
            fp->SetReference( "A0" );
        }
        fp->Reference().SetVisible( false );

        // Draw pads
        uint32_t k_pad = i2D->first_pad_ptr;
        while( IsType( k_pad, 0x32 ) )
        {
            auto* i32 = static_cast<ALLEGRO::T_32_PLACED_PAD<magic>*>( m_ptrs[k_pad] );
            AddPad( &*fp, *i32 );
            k_pad = i32->next;
        }

        // Position the object
        fp->SetPosition( VECTOR2I( Scale( i2D->coords[0] ), Scale( -i2D->coords[1] ) ) );
        fp->SetOrientationDegrees( ( i2D->layer == 0 ? 1. : -1. ) * i2D->rotation / 1000. );
        fp->SetLayerAndFlip( i2D->layer == 0 ? F_Cu : B_Cu );

        // Add annotations
        uint32_t k_marking = i2D->ptr1;
        while( true )
        {
            if( IsType( k_marking, 0x14 ) )
            {
                auto* i14 = static_cast<ALLEGRO::T_14<magic>*>( m_ptrs[k_marking] );
                AddAnnotation( *fp, *i14 );
                k_marking = i14->next;
            }
            else
            {
                break;
            }
        }

        // Add text
        uint32_t k_text = i2D->ptr3;
        while( true )
        {
            if( IsType( k_text, 0x30 ) )
            {
                auto* i30 = static_cast<ALLEGRO::T_30_STRING_GRAPHIC_WRAPPER<magic>*>( m_ptrs[k_text] );
                AddText( *fp, *i30 );
                k_text = i30->next;
            }
            else if( IsType( k_text, 0x03 ) )
            {
                auto* i03 = static_cast<ALLEGRO::T_03<magic>*>( m_ptrs[k_text] );
                // wxLogMessage( "%s T_03.k=%08X", *refdes, local_ntohl( i03->k ) );
                k_text = i03->next;
            }
            else
            {
                break;
            }
        }

        // Add zones
        // wxLogMessage( "Drawing x2D k=0x%08X zones", local_ntohl( i2D->k ) );
        uint32_t k_shape = i2D->ptr4[1];
        while( true )
        {
            if( IsType( k_shape, 0x28 ) )
            {
                auto* i28 = static_cast<ALLEGRO::T_28_SHAPE<magic>*>( m_ptrs[k_shape] );
                // wxLogMessage( "Drawing T_28 t=0x%02X.%02X.%02X k=0x%08X", ntohs( i28->type ),
                //               i28->subtype, i28->layer, local_ntohl( i28->k ) );
                AddZone( *fp, {}, *i28 );
                k_shape = i28->next;
            }
            else if( IsType( k_shape, 0x0E ) )
            {
                auto* i0E = static_cast<ALLEGRO::T_0E<magic>*>( m_ptrs[k_shape] );
                // wxLogMessage( "How to draw T_0E? 0x%08X k=0x%08X", local_ntohl( i0E->t ),
                //               local_ntohl( i0E->k ) );
                k_shape = i0E->next;
            }
            else
            {
                break;
            }
        }

        PCB_FIELD* id_field = fp->GetField( FIELD_T::USER );
        id_field->SetName( "ALLEGRO_ID" );
        id_field->SetText( wxString::Format( "0x%08X", local_ntohl( i2D->k ) ) );
        id_field->SetVisible( false );

        k = i2D->next;

        m_board->Add( fp.release(), ADD_MODE::APPEND );
    }
}

template <ALLEGRO::MAGIC magic>
void ALLEGRO_PARSER<magic>::AddPad( FOOTPRINT* aFootprint, const ALLEGRO::T_32_PLACED_PAD<magic>& aBrdPlacedPad )
{
    std::unique_ptr<PAD> pad = std::make_unique<PAD>( aFootprint );

    // Find net information
    if( IsType( aBrdPlacedPad.net_ptr, 0x04 ) )
    {
        const auto* net_pairing = static_cast<ALLEGRO::T_04_NET_ASSIGNMENT<magic>*>( m_ptrs[aBrdPlacedPad.net_ptr] );

        NETINFO_ITEM* netinfo = NetInfo( *net_pairing );
        pad->SetNet( netinfo );
    }

    const auto* brd_pad = static_cast<ALLEGRO::T_0D_PAD<magic>*>( m_ptrs[aBrdPlacedPad.pad_ptr] );
    VECTOR2I    center;
    center.x = Scale( brd_pad->coords[0] );
    center.y = Scale( -brd_pad->coords[1] );
    pad->SetPosition( center );
    pad->SetOrientationDegrees( brd_pad->rotation / 1000. );

    std::optional<wxString> pad_number = LookupString( brd_pad->str_ptr );
    if( pad_number )
    {
        pad->SetNumber( *pad_number );
    }
    else
    {
        pad->SetNumber( "MISSING" );
    }

    const auto* pad_stack = static_cast<ALLEGRO::T_1C_PAD_STACK<magic>*>( m_ptrs[brd_pad->pad_ptr] );
    // wxLogMessage( "i1C.k = 0x %08X", local_ntohl( i1C->k ) );

    switch( pad_stack->pad_info.pad_type )
    {
    case ALLEGRO::SmtPin:
    case ALLEGRO::SmtPin2:
        pad->SetAttribute( PAD_ATTRIB::SMD );
        pad->SetLayerSet( PAD::SMDMask() );
        break;
    case ALLEGRO::ThroughVia:
    case ALLEGRO::Slot:
        pad->SetAttribute( PAD_ATTRIB::PTH );
        pad->SetLayerSet( PAD::PTHMask() );
        break;
    case ALLEGRO::NonPlatedHole:
        pad->SetAttribute( PAD_ATTRIB::NPTH );
        pad->SetLayerSet( PAD::UnplatedHoleMask() );
        break;
    default: wxLogMessage( "Unknown pad type %d", pad_stack->pad_info.pad_type );
    }

    /*
    const uint32_t component_count = GetPadComponentCount( *i1C );
    wxLogMessage( "next pad, for 0x1C k=0x %08X", local_ntohl( i1C->k ) );
    for( uint8_t i = 0; i < component_count; i++ )
    {
        ALLEGRO::t13<magic>* c = GetPadComponent( *i1C, i );
        wxLogMessage( "%02d: t=0x%08X, ptr=0x%08X", i, c->t, local_ntohl( c->str_ptr ) );
    }
    */

    uint8_t offset = 23;
    if constexpr( magic < ALLEGRO::A_172 )
    {
        offset = 12;
    }
    ALLEGRO::PAD_STACK_COMPONENT<magic>* comp = GetPadComponent( *pad_stack, offset );
    if( comp->t != 0 )
    {
        SetPadShape( *pad, *comp );
        aFootprint->Add( pad.release(), ADD_MODE::APPEND );
    }

    uint8_t mask_offset = 14;
    if constexpr( magic < ALLEGRO::A_172 )
    {
        mask_offset = 0;
    }
    comp = GetPadComponent( *pad_stack, mask_offset );
    if( comp->t != 0 )
    {
        std::unique_ptr<PAD> mask_pad = std::make_unique<PAD>( aFootprint );
        mask_pad->SetAttribute( PAD_ATTRIB::SMD );
        mask_pad->SetLayer( F_Mask );
        mask_pad->SetLayerSet( LSET().set( F_Mask ) );
        mask_pad->SetPosition( center );
        mask_pad->SetOrientationDegrees( brd_pad->rotation / 1000. );
        SetPadShape( *mask_pad, *comp );
        aFootprint->Add( mask_pad.release(), ADD_MODE::APPEND );
    }

    uint8_t paste_offset = 16;
    if constexpr( magic < ALLEGRO::A_172 )
    {
        paste_offset = 5;
    }
    comp = GetPadComponent( *pad_stack, paste_offset );
    if( comp->t != 0 )
    {
        std::unique_ptr<PAD> paste_pad = std::make_unique<PAD>( aFootprint );
        paste_pad->SetAttribute( PAD_ATTRIB::SMD );
        paste_pad->SetLayer( F_Paste );
        paste_pad->SetLayerSet( LSET().set( F_Paste ) );
        paste_pad->SetPosition( center );
        paste_pad->SetOrientationDegrees( brd_pad->rotation / 1000. );
        SetPadShape( *paste_pad, *comp );
        aFootprint->Add( paste_pad.release(), ADD_MODE::APPEND );
    }

    // Add label
    // FIXME: This is positioned incorrectly.
    // if( IsType( i32.ptr10, 0x30 ) )
    // {
    //     ALLEGRO::T_30<magic>* i30 = static_cast<ALLEGRO::T_30<magic>*>( m_ptrs[i32.ptr10] );
    //     AddText( *fp, *i30 );
    // }
}

template <ALLEGRO::MAGIC magic>
void ALLEGRO_PARSER<magic>::AddText( BOARD_ITEM_CONTAINER&                              aContainer,
                                     const ALLEGRO::T_30_STRING_GRAPHIC_WRAPPER<magic>& aStrWrapper )
{
    auto* str_graphic = static_cast<ALLEGRO::T_31_STRING_GRAPHIC<magic>*>( m_ptrs[aStrWrapper.str_graphic_ptr] );

    const char* s = (char*) ( &str_graphic->TAIL );

    if( strlen( s ) == 0 )
    {
        // wxLogMessage( "empty string" );
    }
    else
    {
        std::unique_ptr<PCB_TEXT> t = std::make_unique<PCB_TEXT>( &aContainer );

        // FIXME: This probably needs to be flipped relative to the placement
        PCB_LAYER_ID layer = User_2;
        switch( str_graphic->layer )
        {
        case ALLEGRO::STR_LAYER::TOP_TEXT:
        case ALLEGRO::STR_LAYER::TOP_PIN:
        case ALLEGRO::STR_LAYER::TOP_REFDES: layer = F_SilkS; break;
        case ALLEGRO::STR_LAYER::BOT_TEXT:
        case ALLEGRO::STR_LAYER::BOT_PIN:
        case ALLEGRO::STR_LAYER::BOT_REFDES: layer = B_SilkS; break;
        case ALLEGRO::STR_LAYER::TOP_PIN_LABEL: layer = User_5; break;
        }

        t->SetLayer( layer );
        t->SetVertJustify( GR_TEXT_V_ALIGN_BOTTOM );
        t->SetPosition( VECTOR2I( Scale( aStrWrapper.coords[0] ), Scale( -aStrWrapper.coords[1] ) ) );
        t->Rotate( t->GetPosition(), EDA_ANGLE( aStrWrapper.rotation / 1000., DEGREES_T ) );
        t->SetText( wxString( s ) );

        ALLEGRO::TEXT_PROPERTIES props;
        if constexpr( !std::is_same_v<decltype( aStrWrapper.font ), std::monostate> )
        {
            props = aStrWrapper.font;
        }
        else
        {
            props = aStrWrapper.font_16x;
        }

        switch( props.align )
        {
        case ALLEGRO::TEXT_ALIGNMENT::TextAlignLeft: t->SetHorizJustify( GR_TEXT_H_ALIGN_LEFT ); break;
        case ALLEGRO::TEXT_ALIGNMENT::TextAlignCenter: t->SetHorizJustify( GR_TEXT_H_ALIGN_CENTER ); break;
        case ALLEGRO::TEXT_ALIGNMENT::TextAlignRight: t->SetHorizJustify( GR_TEXT_H_ALIGN_RIGHT ); break;
        }

        if( props.reversed == ALLEGRO::TEXT_REVERSAL::TextReversed )
        {
            t->SetMirrored( true );
        }

        const ALLEGRO::T_36_08<magic>* font = FindFont( props.key );
        if( font != nullptr )
        {
            t->SetTextSize( VECTOR2I( Scale( font->char_width ), Scale( font->char_height ) ) );
        }
        else
        {
            wxLogMessage( "Font == null" );
        }

        aContainer.Add( t.release(), ADD_MODE::APPEND );
    }
}

template <ALLEGRO::MAGIC magic>
void ALLEGRO_PARSER<magic>::AddTrack( const ALLEGRO::T_1B_NET<magic>&   aBrdNet,
                                      const ALLEGRO::T_05_TRACK<magic>& aBrdTrack )
{
    uint32_t k = aBrdTrack.first_segment_ptr;

    NETINFO_ITEM* netinfo = NetInfo( aBrdNet.net_name );
    PCB_LAYER_ID  layer = EtchLayerToKi( aBrdTrack.layer );

    while( true )
    {
        if( IsType( k, 0x01 ) )
        {
            auto* i01 = static_cast<ALLEGRO::T_01_ARC<magic>*>( m_ptrs[k] );

            double r = cfp_to_double( i01->r );

            auto [start, end] = GetCoordinatePair( *i01 );
            VECTOR2D center{ Scale( (int32_t) cfp_to_double( i01->x ) ), Scale( -(int32_t) cfp_to_double( i01->y ) ) };
            VECTOR2I mid = CalcArcMid( start, end, center );

            SHAPE_ARC shape_arc;
            shape_arc.ConstructFromStartEndCenter( start, end, center, i01->subtype == 0x00, 0 );

            std::unique_ptr<PCB_ARC> arc = std::make_unique<PCB_ARC>( m_board, &shape_arc );

            arc->SetLayer( layer );
            arc->SetWidth( Scale( i01->width ) );
            arc->SetNet( netinfo );

            m_board->Add( arc.release(), ADD_MODE::APPEND );

            k = i01->next;
        }
        else if( IsType( k, 0x15 ) )
        {
            auto* i15 = static_cast<ALLEGRO::T_15_SEGMENT<magic>*>( m_ptrs[k] );
            auto [start, end] = GetCoordinatePair( *i15 );
            std::unique_ptr<PCB_TRACK> track = std::make_unique<PCB_TRACK>( m_board );

            track->SetLayer( layer );
            track->SetWidth( Scale( i15->width ) );
            track->SetStart( start );
            track->SetEnd( end );
            track->SetNet( netinfo );

            m_board->Add( track.release(), ADD_MODE::APPEND );

            k = i15->next;
        }
        else if( IsType( k, 0x16 ) )
        {
            auto* i16 = static_cast<ALLEGRO::T_16_SEGMENT<magic>*>( m_ptrs[k] );
            auto [start, end] = GetCoordinatePair( *i16 );
            std::unique_ptr<PCB_TRACK> track = std::make_unique<PCB_TRACK>( m_board );

            track->SetLayer( layer );
            track->SetWidth( Scale( i16->width ) );
            track->SetStart( start );
            track->SetEnd( end );
            track->SetNet( netinfo );

            m_board->Add( track.release(), ADD_MODE::APPEND );

            k = i16->next;
        }
        else if( IsType( k, 0x17 ) )
        {
            auto* i17 = static_cast<ALLEGRO::T_17_SEGMENT<magic>*>( m_ptrs[k] );
            auto [start, end] = GetCoordinatePair( *i17 );
            std::unique_ptr<PCB_TRACK> track = std::make_unique<PCB_TRACK>( m_board );

            track->SetLayer( layer );
            track->SetWidth( Scale( i17->width ) );
            track->SetStart( start );
            track->SetEnd( end );
            track->SetNet( netinfo );

            m_board->Add( track.release(), ADD_MODE::APPEND );

            k = i17->next;
        }
        else
        {
            break;
        }
    }
}

template <ALLEGRO::MAGIC magic>
void ALLEGRO_PARSER<magic>::AddVia( const ALLEGRO::T_33_VIA<magic>& aBrdVia )
{
    std::unique_ptr<PCB_VIA> via = std::make_unique<PCB_VIA>( m_board );
    via->SetPosition( VECTOR2D( Scale( aBrdVia.coords[0] ), Scale( -aBrdVia.coords[1] ) ) );

    // Find net information
    if( IsType( aBrdVia.net_ptr, 0x04 ) )
    {
        const auto*   i04 = static_cast<ALLEGRO::T_04_NET_ASSIGNMENT<magic>*>( m_ptrs[aBrdVia.net_ptr] );
        NETINFO_ITEM* netinfo = NetInfo( *i04 );
        via->SetNet( netinfo );
    }

    m_board->Add( via.release(), ADD_MODE::APPEND );
}

template <ALLEGRO::MAGIC magic>
void ALLEGRO_PARSER<magic>::AddZone( BOARD_ITEM_CONTAINER&                          aContainer,
                                     const std::optional<ALLEGRO::T_1B_NET<magic>>& aBrdNet,
                                     const ALLEGRO::T_28_SHAPE<magic>&              aBrdShape )
{
    uint32_t k = aBrdShape.first_segment_ptr;

    std::unique_ptr<SHAPE_POLY_SET> outline = std::make_unique<SHAPE_POLY_SET>();
    outline->AddOutline( ShapeStartingAt( &k ) );

    uint32_t cutout_k = aBrdShape.cutouts_ptr;
    while( IsType( cutout_k, 0x34 ) )
    {
        auto* i34 = static_cast<ALLEGRO::T_34_RULE_REGION<magic>*>( m_ptrs[cutout_k] );
        // wxLogMessage( "Handling x34 0x%08X", local_ntohl( i34->k ) );
        uint32_t k = i34->first_segment_ptr;

        SHAPE_LINE_CHAIN cutout_chain = ShapeStartingAt( &k );
        if( cutout_chain.PointCount() >= 3 )
        {
            cutout_chain.SetClosed( true );
            outline->AddHole( cutout_chain );
        }
        else
        {
            // wxLogMessage( "Skipping, why?" );
        }
        cutout_k = i34->next;
    }

    if( aBrdShape.layer.family == ALLEGRO::LAYER_FAMILY::COPPER )
    {
        std::unique_ptr<ZONE> zone = std::make_unique<ZONE>( &aContainer );
        zone->SetZoneName( wxString::Format( "x28: 0x%08X", local_ntohl( aBrdShape.k ) ) );
        zone->SetLocalClearance( 0 );
        zone->SetMinThickness( 0 );
        zone->SetOutline( outline.release() );
        zone->SetPadConnection( ZONE_CONNECTION::INHERITED );

        PCB_LAYER_ID layer = EtchLayerToKi( aBrdShape.layer );

        zone->SetLayer( layer );

        if( aBrdNet )
        {
            NETINFO_ITEM* netinfo = NetInfo( aBrdNet->net_name );
            zone->SetNet( netinfo );
        }

        // zone->SetFilledPolysList( layer, outline );

        aContainer.Add( zone.release(), ADD_MODE::APPEND );
    }
    else if( aBrdShape.layer.family == ALLEGRO::LAYER_FAMILY::BOARD_GEOMETRY && aBrdShape.layer.ordinal == 0xFD )
    {
        std::unique_ptr<PCB_SHAPE> shape = std::make_unique<PCB_SHAPE>( &aContainer, SHAPE_T::POLY );
        shape->SetPolyShape( *outline.release() );
        shape->SetFilled( false );
        shape->SetLayer( Edge_Cuts );
        aContainer.Add( shape.release(), ADD_MODE::APPEND );
    }
    else if( aBrdShape.layer.family == ALLEGRO::LAYER_FAMILY::SILK )
    {
        std::unique_ptr<PCB_SHAPE> shape = std::make_unique<PCB_SHAPE>( &aContainer, SHAPE_T::POLY );
        shape->SetPolyShape( *outline.release() );
        shape->SetFilled( true );

        if( auto result = LookupPcbLayer( aBrdShape.layer ); result.has_value() )
        {
            shape->SetLayer( *result );
        }
        else if( aBrdShape.layer.ordinal == 0xEC )
        {
            shape->SetWidth( 0 );
            shape->SetLayer( B_Mask );
        }
        else if( aBrdShape.layer.ordinal == 0xED )
        {
            shape->SetWidth( 0 );
            shape->SetLayer( F_Mask );
        }
        else if( aBrdShape.layer.ordinal == 0xF6 )
        {
            shape->SetFilled( false );
            shape->SetLayer( F_SilkS );
        }
        else if( aBrdShape.layer.ordinal == 0xF7 )
        {
            shape->SetLayer( User_6 );
        }
        else if( aBrdShape.layer.ordinal == 0xEE )
        {
            // What are these?
            shape->SetFilled( false );
            shape->SetLayer( User_7 );
        }
        else if( aBrdShape.layer.ordinal == 0xEF )
        {
            // What are these?
            shape->SetFilled( false );
            shape->SetLayer( User_8 );
        }
        else if( aBrdShape.layer.ordinal == 0x02 )
        {
            shape->SetFilled( false );
            shape->SetLayer( Eco1_User );
        }
        else if( aBrdShape.layer.ordinal == 0x00 )
        {
            shape->SetFilled( false );
            shape->SetLayer( Eco2_User );
        }
        else
        {
            // wxLogMessage( "Adding zone from 09.%02X", i28.layer );
            shape->SetLayer( User_3 );
        }

        aContainer.Add( shape.release(), ADD_MODE::APPEND );
    }
    /*
    else if( i28.layer.family == ALLEGRO::ZONE_TYPE_TBD )

    {
        // PCB_LAYER_ID layer = (PCB_LAYER_ID) ( ( (int) F_Cu ) + i28.layer );
        zone->SetLayer( F_Cu );

        // zone->SetIsRuleArea( true );
        // zone->SetDoNotAllowCopperPour( true );

        aContainer.Add( zone.release(), ADD_MODE::APPEND );
    }
    */
    else
    {
        // zone->SetLayer( User_3 );
        // aContainer.Add( zone.release(), ADD_MODE::APPEND );
    }
}

// This is typically used to frame things, like tables or pages.
template <ALLEGRO::MAGIC magic>
void ALLEGRO_PARSER<magic>::AddRectangle( BOARD_ITEM_CONTAINER&                 aContainer,
                                          const ALLEGRO::T_24_RECTANGLE<magic>& aBrdRect )
{
    std::unique_ptr<SHAPE_POLY_SET> outline = std::make_unique<SHAPE_POLY_SET>();

    SHAPE_LINE_CHAIN chain;
    auto [a, b] = GetCoordinatePair( aBrdRect );
    chain.Append( a.x, a.y );
    chain.Append( a.x, b.y );
    chain.Append( b.x, b.y );
    chain.Append( b.x, a.y );
    chain.SetClosed( true );

    outline->AddOutline( chain );

    std::unique_ptr<PCB_SHAPE> shape = std::make_unique<PCB_SHAPE>( &aContainer, SHAPE_T::POLY );
    shape->SetPolyShape( *outline.release() );
    shape->SetFilled( false );
    shape->SetLayer( LookupPcbLayer( aBrdRect.layer ).value_or( User_3 ) );
    aContainer.Add( shape.release(), ADD_MODE::APPEND );
}

template <ALLEGRO::MAGIC magic>
void ALLEGRO_PARSER<magic>::BuildBoard()
{
    UpdateLayerInfo();

    auto* net = static_cast<ALLEGRO::T_1B_NET<magic>*>( m_ptrs[m_header->ll_x1B.head] );
    if( net != nullptr )
    {
        do
        {
            if( net->ptr1 != 0 )
            {
                auto* i04 = static_cast<ALLEGRO::T_04_NET_ASSIGNMENT<magic>*>( m_ptrs[net->ptr1] );
                do
                {
                    uint32_t k = i04->ptr2;
                    while( true )
                    {
                        if( IsType( k, 0x33 ) )
                        {
                            auto* i33 = static_cast<ALLEGRO::T_33_VIA<magic>*>( m_ptrs[k] );
                            // auto& i = fs->get_x33( k );
                            // printf("- - Found x33 w/ key = 0x %08X\n", local_ntohl(k));
                            AddVia( *i33 );
                            k = i33->un1;
                        }
                        else if( IsType( k, 0x32 ) )
                        {
                            auto* i = static_cast<ALLEGRO::T_32_PLACED_PAD<magic>*>( m_ptrs[k] );
                            // printf("- - Found x32 w/ key = 0x %08X\n", local_ntohl(k));
                            k = i->un1;
                        }
                        else if( IsType( k, 0x2E ) )
                        {
                            auto* i = static_cast<ALLEGRO::T_2E<magic>*>( m_ptrs[k] );
                            // printf("- - Found x2E w/ key = 0x %08X\n", local_ntohl(k));
                            k = i->un[0];
                        }
                        else if( IsType( k, 0x28 ) )
                        {
                            auto* i28 = static_cast<ALLEGRO::T_28_SHAPE<magic>*>( m_ptrs[k] );
                            AddZone( *m_board, *net, *i28 );

                            k = i28->next;
                        }
                        else if( IsType( k, 0x0E ) )
                        {
                            auto* i = static_cast<ALLEGRO::T_0E<magic>*>( m_ptrs[k] );
                            k = i->un[0];
                        }
                        else if( IsType( k, 0x05 ) )
                        {
                            auto* i05 = static_cast<ALLEGRO::T_05_TRACK<magic>*>( m_ptrs[k] );
                            AddTrack( *net, *i05 );

                            k = i05->ptr0;
                        }
                        else if( IsType( k, 0x04 ) | IsType( k, 0x1B ) )
                        {
                            break;
                        }
                        else
                        {
                            // wxLogMessage( "Unexpected key = 0x %08X :(", local_ntohl( k ) );
                            break;
                        }
                    }

                    if( i04->next == net->k )
                    {
                        break;
                    }
                    else
                    {
                        i04 = static_cast<ALLEGRO::T_04_NET_ASSIGNMENT<magic>*>( m_ptrs[i04->next] );
                    }
                } while( true );
            }

            net = static_cast<ALLEGRO::T_1B_NET<magic>*>( m_ptrs[net->next] );
        } while( net->next != m_header->ll_x1B.tail );
    }

    uint32_t k = m_header->ll_x0E_x28.head;
    if( k != 0 )
    {
        while( k != m_header->ll_x0E_x28.tail )
        {
            if( IsType( k, 0x0E ) )
            {
                // Do not know what this represents
                auto i0E = static_cast<ALLEGRO::T_0E<magic>*>( m_ptrs[k] );
                k = i0E->next;
            }
            else if( IsType( k, 0x28 ) )
            {
                auto i28 = static_cast<ALLEGRO::T_28_SHAPE<magic>*>( m_ptrs[k] );
                AddZone( *m_board, {}, *i28 );
                k = i28->next;
            }
            else
            {
                // wxLogMessage( "Unexpected type with key = 0x%08X!", local_ntohl( k ) );
                break;
            }
        }
    }

    k = m_header->ll_x14.head;
    if( k != 0 )
    {
        while( k != m_header->ll_x14.tail )
        {
            auto i14 = static_cast<ALLEGRO::T_14<magic>*>( m_ptrs[k] );
            AddAnnotation( *m_board, *i14 );
            k = i14->next;
        }
    }

    k = m_header->ll_x2B.head;
    if( k != 0 )
    {
        while( k != m_header->ll_x2B.tail )
        {
            auto i2B = static_cast<ALLEGRO::T_2B_FOOTPRINT<magic>*>( m_ptrs[k] );
            AddFootprint( *i2B );
            k = i2B->next;
        }
    }

    k = m_header->ll_x03_x30.head;
    if( k != 0 )
    {
        while( k != m_header->ll_x03_x30.tail )
        {
            if( IsType( k, 0x30 ) )
            {
                auto* i30 = static_cast<ALLEGRO::T_30_STRING_GRAPHIC_WRAPPER<magic>*>( m_ptrs[k] );
                AddText( *m_board, *i30 );
                k = i30->next;
            }
            else if( IsType( k, 0x03 ) )
            {
                auto* i03 = static_cast<ALLEGRO::T_03<magic>*>( m_ptrs[k] );
                k = i03->next;
            }
            else
            {
                // wxLogMessage( "Unexpected type in ll_x03_x30" );
                break;
            }
        }
    }

    k = m_header->ll_x24_x28.head;
    if( k != 0 )
    {
        while( k != m_header->ll_x24_x28.tail )
        {
            if( IsType( k, 0x24 ) )
            {
                auto* i = static_cast<ALLEGRO::T_24_RECTANGLE<magic>*>( m_ptrs[k] );
                AddRectangle( *m_board, *i );
                k = i->next;
            }
            else if( IsType( k, 0x28 ) )
            {
                auto* i = static_cast<ALLEGRO::T_28_SHAPE<magic>*>( m_ptrs[k] );
                AddZone( *m_board, {}, *i );
                k = i->next;
            }
            else
            {
                THROW_IO_ERROR(
                        wxString::Format( "Expected t=0x24 or t=0x28 but k=0x%08X unexpected", local_ntohl( k ) ) );
            }
        }
    }
}

template <ALLEGRO::MAGIC magic>
void ALLEGRO_PARSER<magic>::Skip( std::size_t aSkipBytes )
{
    m_curAddr = (void*) ( ( (char*) m_curAddr ) + aSkipBytes );
}

template <ALLEGRO::MAGIC magic>
void ALLEGRO_PARSER<magic>::Log( const char* fmt... )
{
    va_list args;
    va_start( args, fmt );

    wxLogMessage( "@ 0x%08lX:", (char*) m_curAddr - (char*) m_baseAddr );
    wxVLogMessage( fmt, args );

    va_end( args );
}

template <ALLEGRO::MAGIC magic>
bool ALLEGRO_PARSER<magic>::IsType( uint32_t aKey, uint8_t aType )
{
    return ( m_ptrs.count( aKey ) > 0 ) && ( *(uint8_t*) m_ptrs[aKey] == aType );
}

template <ALLEGRO::MAGIC magic>
std::optional<wxString> ALLEGRO_PARSER<magic>::LookupString( uint32_t aKey ) const
{
    if( m_strings.count( aKey ) > 0 )
    {
        return wxString( m_strings.at( aKey ) );
    }
    return {};
}

template <ALLEGRO::MAGIC magic>
std::optional<PCB_LAYER_ID> ALLEGRO_PARSER<magic>::LookupPcbLayer( const ALLEGRO::LAYER_INFO& aLayer ) const
{
    if( auto kv = ALLEGRO::LAYER_MAPPING.find( aLayer ); kv != ALLEGRO::LAYER_MAPPING.end() )
    {
        return kv->second;
    }
    else
    {
        return {};
    }
}

template <ALLEGRO::MAGIC magic>
std::optional<wxString> ALLEGRO_PARSER<magic>::LookupBrdLayerName( const ALLEGRO::LAYER_INFO& aLayer ) const
{
    uint32_t ptr = m_header->layer_sets.at( static_cast<uint8_t>( aLayer.family ) ).layer_set_ptr;
    if( m_layerSetMap.count( ptr ) > 0 )
    {
        const ALLEGRO::T_2A_LAYER_SET<magic>* layer_set = &m_layerSetMap.at( ptr );
        if constexpr( magic <= ALLEGRO::A_164 )
        {
            if( layer_set->local_entries.size() > aLayer.ordinal )
            {
                return layer_set->local_entries[aLayer.ordinal].layer_name;
            }
        }
        else
        {
            if( layer_set->reference_entries.size() > aLayer.ordinal )
            {
                return LookupString( layer_set->reference_entries[aLayer.ordinal].layer_name_str_ptr );
            }
        }
    }
    return {};
}

template <ALLEGRO::MAGIC magic>
SHAPE_LINE_CHAIN ALLEGRO_PARSER<magic>::ShapeStartingAt( uint32_t* aKey )
{
    SHAPE_LINE_CHAIN chain;

    bool first = true;
    while( aKey != nullptr )
    {
        if( IsType( *aKey, 0x01 ) )
        {
            auto* i01 = static_cast<ALLEGRO::T_01_ARC<magic>*>( m_ptrs[*aKey] );

            auto [start, end] = GetCoordinatePair( *i01 );
            VECTOR2D center{ Scale( (int32_t) cfp_to_double( i01->x ) ), Scale( -(int32_t) cfp_to_double( i01->y ) ) };

            SHAPE_ARC arc;
            arc.ConstructFromStartEndCenter( start, end, center, i01->subtype == 0x00, 0 );

            if( first )
            {
                chain.Append( start );
            }
            chain.Append( arc );

            *aKey = i01->next;
        }
        else if( IsType( *aKey, 0x15 ) )
        {
            auto* segment_inst = static_cast<ALLEGRO::T_15_SEGMENT<magic>*>( m_ptrs[*aKey] );
            AddSegmentToChain( chain, segment_inst, first );
            *aKey = segment_inst->next;
        }
        else if( IsType( *aKey, 0x16 ) )
        {
            auto* segment_inst = static_cast<ALLEGRO::T_16_SEGMENT<magic>*>( m_ptrs[*aKey] );
            AddSegmentToChain( chain, segment_inst, first );
            *aKey = segment_inst->next;
        }
        else if( IsType( *aKey, 0x17 ) )
        {
            auto* segment_inst = static_cast<ALLEGRO::T_17_SEGMENT<magic>*>( m_ptrs[*aKey] );
            AddSegmentToChain( chain, segment_inst, first );
            *aKey = segment_inst->next;
        }
        else
        {
            break;
        }

        first = false;
    }

    chain.SetClosed( true );
    return chain;
}

template <ALLEGRO::MAGIC magic>
template <class SEGMENT_TYPE>
void ALLEGRO_PARSER<magic>::AddSegmentToChain( SHAPE_LINE_CHAIN& aChain, SEGMENT_TYPE* aSegment, bool aFirstSegment )
{
    auto [start, end] = GetCoordinatePair( *aSegment );
    if( aFirstSegment )
    {
        aChain.Append( start );
    }
    aChain.Append( end );
}

template <ALLEGRO::MAGIC magic>
PCB_LAYER_ID ALLEGRO_PARSER<magic>::EtchLayerToKi( const ALLEGRO::LAYER_INFO& aLayer )
{
    if( aLayer.ordinal == 0 )
    {
        return F_Cu;
    }
    else if( aLayer.ordinal == m_layerCount - 1 )
    {
        return B_Cu;
    }
    else
    {
        return static_cast<PCB_LAYER_ID>( B_Cu + ( aLayer.ordinal * 2 ) );
    }
}

template <ALLEGRO::MAGIC magic>
double ALLEGRO_PARSER<magic>::Scale( double aBrdValue )
{
    return aBrdValue * m_scaleFactor;
}

template <ALLEGRO::MAGIC magic>
template <class T>
std::pair<VECTOR2D, VECTOR2D> ALLEGRO_PARSER<magic>::GetCoordinatePair( T& aInst )
{
    return { { Scale( aInst.coords[0] ), Scale( -aInst.coords[1] ) },
             { Scale( aInst.coords[2] ), Scale( -aInst.coords[3] ) } };
}

template <ALLEGRO::MAGIC magic>
const ALLEGRO::T_36_08<magic>* ALLEGRO_PARSER<magic>::FindFont( uint8_t aKey )
{
    // Unclear why there's an offset, but this seems to make things
    // work nicely?
    constexpr int8_t offset = -1;

    for( const auto& [_k, x36_inst] : this->m_t36_map )
    {
        // FIXME: Throw an error if `k` is larger than the size of the list
        if( x36_inst->c == 0x08 )
        {
            return reinterpret_cast<ALLEGRO::T_36_08<magic>*>(
                    ( (char*) x36_inst ) + ALLEGRO::sizeof_allegro_obj<ALLEGRO::T_36<magic>>()
                    + ALLEGRO::sizeof_allegro_obj<ALLEGRO::T_36_08<magic>>() * ( aKey + offset ) );
        }
    }

    wxLogMessage( "Failed to find font" );
    return nullptr;
}

template <ALLEGRO::MAGIC magic>
constexpr uint32_t ALLEGRO_PARSER<magic>::GetPadComponentCount( const ALLEGRO::T_1C_PAD_STACK<magic>& aPadStack )
{
    if( magic < ALLEGRO::A_172 )
    {
        return 10 + 3 * aPadStack.layer_count;
    }
    else
    {
        return 21 + 4 * aPadStack.layer_count;
    }
}

template <ALLEGRO::MAGIC magic>
void ALLEGRO_PARSER<magic>::SetPadShape( PAD& aPad, const ALLEGRO::PAD_STACK_COMPONENT<magic>& aPadStackComponent )
{
    switch( aPadStackComponent.t )
    {
    case 0x02: aPad.SetShape( PADSTACK::ALL_LAYERS, PAD_SHAPE::CIRCLE ); break;
    case 0x05:
    case 0x06: aPad.SetShape( PADSTACK::ALL_LAYERS, PAD_SHAPE::RECTANGLE ); break;
    case 0x0B:
    case 0x1B:
    case 0x0C: aPad.SetShape( PADSTACK::ALL_LAYERS, PAD_SHAPE::ROUNDRECT ); break;
    case 0x16: aPad.SetShape( PADSTACK::ALL_LAYERS, PAD_SHAPE::CUSTOM ); break;
    default: wxLogMessage( "Unrecognized type: t=%02X", aPadStackComponent.t );
    }

    if( aPadStackComponent.t == 0x16 )
    {
        if( IsType( aPadStackComponent.str_ptr, 0x28 ) )
        {
            auto*    i28 = static_cast<ALLEGRO::T_28_SHAPE<magic>*>( m_ptrs[aPadStackComponent.str_ptr] );
            uint32_t k = i28->first_segment_ptr;
            aPad.DeletePrimitivesList();
            aPad.AddPrimitivePoly( PADSTACK::ALL_LAYERS, ShapeStartingAt( &k ), 0, true );

            // FIXME: Set the fake pad to a very small size
            aPad.SetSize( PADSTACK::ALL_LAYERS, VECTOR2I( pcbIUScale.MilsToIU( 1 ), pcbIUScale.MilsToIU( 1 ) ) );
        }
        else
        {
            wxLogMessage( "Weird case?, str_ptr = 0x %08X, count = %d", local_ntohl( aPadStackComponent.str_ptr ),
                          m_ptrs.count( aPadStackComponent.str_ptr ) );
        }
    }
    else
    {
        aPad.SetSize( PADSTACK::ALL_LAYERS, VECTOR2I( Scale( aPadStackComponent.w ), Scale( aPadStackComponent.h ) ) );
    }
    aPad.Move( VECTOR2I( Scale( aPadStackComponent.x3 ), -Scale( aPadStackComponent.x4 ) ) );
}

template <ALLEGRO::MAGIC magic>
constexpr ALLEGRO::PAD_STACK_COMPONENT<magic>*
ALLEGRO_PARSER<magic>::GetPadComponent( const ALLEGRO::T_1C_PAD_STACK<magic>& aPadStack, uint32_t aIndex )
{
    return (ALLEGRO::PAD_STACK_COMPONENT<magic>*) ( (char*) &aPadStack
                                                    + ALLEGRO::sizeof_allegro_obj<ALLEGRO::T_1C_PAD_STACK<magic>>()
                                                    + ALLEGRO::sizeof_allegro_obj<ALLEGRO::PAD_STACK_COMPONENT<magic>>()
                                                              * aIndex );
}

template <ALLEGRO::MAGIC magic>
std::optional<wxString>
ALLEGRO_PARSER<magic>::RefdesLookup( const ALLEGRO::T_2D_PLACED_FOOTPRINT<magic>& aPlacedFootprint )
{
    uint32_t inst_ref = 0;
    if constexpr( std::is_same_v<decltype( aPlacedFootprint.inst_ref ), std::monostate> )
    {
        inst_ref = aPlacedFootprint.inst_ref_16x;
    }
    else
    {
        inst_ref = aPlacedFootprint.inst_ref;
    }

    if( inst_ref == 0 || !IsType( inst_ref, 0x07 ) )
        return {};

    auto* i07 = static_cast<ALLEGRO::T_07<magic>*>( m_ptrs[inst_ref] );
    wxASSERT_MSG( m_strings.count( i07->refdes_string_ref ) == 1, "Expected a string" );
    return wxString( m_strings[i07->refdes_string_ref] );
}

template <ALLEGRO::MAGIC magic>
NETINFO_ITEM* ALLEGRO_PARSER<magic>::NetInfo( uint32_t aKey )
{
    NETINFO_ITEM*           netinfo = nullptr;
    std::optional<wxString> netname = LookupString( aKey );

    if( netname )
    {
        if( NETINFO_ITEM* item = m_board->FindNet( *netname ) )
        {
            netinfo = item;
        }
        else
        {
            item = new NETINFO_ITEM( m_board, *netname, m_board->GetNetCount() + 1 );
            m_board->Add( item, ADD_MODE::APPEND );
            netinfo = item;
        }
    }
    return netinfo;
}

template <ALLEGRO::MAGIC magic>
NETINFO_ITEM* ALLEGRO_PARSER<magic>::NetInfo( const ALLEGRO::T_04_NET_ASSIGNMENT<magic>& aNetAssignment )
{
    auto* i1B = static_cast<ALLEGRO::T_1B_NET<magic>*>( m_ptrs[aNetAssignment.ptr1] );
    return NetInfo( i1B->net_name );
}

template class ALLEGRO_PARSER<ALLEGRO::A_160>;
template class ALLEGRO_PARSER<ALLEGRO::A_162>;
template class ALLEGRO_PARSER<ALLEGRO::A_164>;
template class ALLEGRO_PARSER<ALLEGRO::A_165>;
template class ALLEGRO_PARSER<ALLEGRO::A_166>;
template class ALLEGRO_PARSER<ALLEGRO::A_172>;
template class ALLEGRO_PARSER<ALLEGRO::A_174>;
template class ALLEGRO_PARSER<ALLEGRO::A_175>;
