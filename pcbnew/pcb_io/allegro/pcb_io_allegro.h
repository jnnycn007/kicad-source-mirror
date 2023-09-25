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

#ifndef ALLEGRO_PLUGIN_H_
#define ALLEGRO_PLUGIN_H_

#include <wx/log.h>
#include <pcb_io/pcb_io.h>
#include <pcb_io/common/plugin_common_layer_mapping.h>

class PCB_IO_ALLEGRO : public PCB_IO, public LAYER_MAPPABLE_PLUGIN
{
public:
    const IO_BASE::IO_FILE_DESC GetBoardFileDesc() const override
    {
        return IO_BASE::IO_FILE_DESC( _HKI( "Cadence Allegro layout file" ), { "brd" } );
    }

    const IO_BASE::IO_FILE_DESC GetLibraryDesc() const override { return GetBoardFileDesc(); }

    long long GetLibraryTimestamp( const wxString& aLibraryPath ) const override { return 0; }

    bool CanReadBoard( const wxString& aFileName ) const override;
    bool CanReadFootprint( const wxString& aFileName ) const override { return false; };

    // FIXME: Should we return true if we can read all the library elements of
    // this file?
    bool CanReadLibrary( const wxString& aFileName ) const override { return false; }

    std::vector<FOOTPRINT*> GetImportedCachedLibraryFootprints() override { return {}; }

    BOARD* LoadBoard( const wxString& aFileName, BOARD* aAppendToMe,
                      const std::map<std::string, UTF8>* aProperties = nullptr,
                      PROJECT*                           aProject = nullptr ) override;

    PCB_IO_ALLEGRO();
    ~PCB_IO_ALLEGRO() override;

private:
    BOARD* m_board;
};

#endif // ALLEGRO_PLUGIN_H_
