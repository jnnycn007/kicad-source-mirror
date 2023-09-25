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

#include "pcb_io_allegro.h"

#include <wx/wfstream.h>
#include <wx/log.h>
#include <wx/translation.h>

#include <board.h>

#include "allegro_file.h"
#include "allegro_pcb.h"
#include "allegro_structs.h"

PCB_IO_ALLEGRO::PCB_IO_ALLEGRO() : PCB_IO( wxS( "Cadence Allegro" ) ), m_board( nullptr ) {};

PCB_IO_ALLEGRO::~PCB_IO_ALLEGRO() = default;

bool PCB_IO_ALLEGRO::CanReadBoard( const wxString& aFileName ) const
{
    if( !PCB_IO::CanReadBoard( aFileName ) )
        return false;

    wxFileInputStream input( aFileName );
    if( input.IsOk() && !input.Eof() )
    {
        uint32_t magic = 0;
        if( input.GetLength() < sizeof( magic ) )
        {
            return false;
        }

        if( !input.ReadAll( &magic, sizeof( magic ) ) )
        {
            return false;
        }

        if( ALLEGRO::A_160 <= magic && magic <= ALLEGRO::A_MAX )
        {
            return true;
        }
    }

    return false;
};

BOARD* PCB_IO_ALLEGRO::LoadBoard( const wxString& aFileName, BOARD* aAppendToMe,
                                  const std::map<std::string, UTF8>* aProperties,
                                  PROJECT*                           aProject )
{
    // FIXME: What is the intent of appending to a board?
    m_board = aAppendToMe ? aAppendToMe : new BOARD();

    if( !aAppendToMe )
    {
        m_board->SetFileName( aFileName );
    }

    ALLEGRO_FILE allegroBrdFile( aFileName );
    ALLEGRO_PCB  pcb( m_board );
    pcb.Parse( allegroBrdFile );

    // this->m_layer_mapping_handler( std::vector<INPUT_LAYER_DESC>() );

    return m_board;
}
