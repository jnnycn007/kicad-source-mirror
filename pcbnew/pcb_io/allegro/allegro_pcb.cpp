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

#include "allegro_pcb.h"

#include <wx/log.h>

#include "allegro_parser.h"

ALLEGRO_PCB::ALLEGRO_PCB( BOARD* aBoard ) : m_board( aBoard )
{
}

void ALLEGRO_PCB::Parse( const ALLEGRO_FILE& aAllegroBrdFile )
{
    uint32_t magic = *(uint32_t*) ( aAllegroBrdFile.region.get_address() );
    if( const auto& result = PARSERS.find( magic ); result != PARSERS.end() )
    {
        result->second( m_board, aAllegroBrdFile );
    }
    else
    {
        THROW_IO_ERROR( wxString::Format( _( "Board file magic=0x%08X not recognized." ), local_ntohl( magic ) ) );
    }
}
