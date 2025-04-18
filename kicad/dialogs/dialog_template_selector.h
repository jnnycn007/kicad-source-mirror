/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2012 Brian Sidebotham <brian.sidebotham@gmail.com>
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

#ifndef PROJECT_TEMPLATE_SELECTOR_H
#define PROJECT_TEMPLATE_SELECTOR_H

#include <dialogs/dialog_template_selector_base.h>
#include "project_template.h"

#include <map>

class DIALOG_TEMPLATE_SELECTOR;

class TEMPLATE_WIDGET : public TEMPLATE_WIDGET_BASE
{
public:
    TEMPLATE_WIDGET( wxWindow* aParent, DIALOG_TEMPLATE_SELECTOR* aDialog );

    /**
     * Set the project template for this widget, which will determine the icon and title
     * associated with this project template widget
     */
    void SetTemplate(PROJECT_TEMPLATE* aTemplate);

    PROJECT_TEMPLATE* GetTemplate() { return m_currTemplate; }

    void Select();
    void Unselect();

protected:
    void OnKillFocus( wxFocusEvent& event );
    void OnMouse( wxMouseEvent& event );

private:
    bool IsSelected() { return m_selected; }

protected:
    DIALOG_TEMPLATE_SELECTOR* m_dialog;
    wxWindow*                 m_parent;
    wxPanel*                  m_panel;
    bool                      m_selected;

    PROJECT_TEMPLATE*         m_currTemplate;
};


class TEMPLATE_SELECTION_PANEL : public TEMPLATE_SELECTION_PANEL_BASE
{
public:
    /**
     * @param aParent The window creating the dialog
     * @param aPath the path
     */
    TEMPLATE_SELECTION_PANEL( wxNotebookPage* aParent, const wxString& aPath );

    const wxString& GetPath() { return m_templatesPath; }

    void AddTemplateWidget( TEMPLATE_WIDGET* aTemplateWidget );

protected:
    wxNotebookPage* m_parent;
    wxString        m_templatesPath;   ///< the path to access to the folder
                                       ///<   containing the templates (which are also folders)
};


class DIALOG_TEMPLATE_SELECTOR : public DIALOG_TEMPLATE_SELECTOR_BASE
{
public:
    DIALOG_TEMPLATE_SELECTOR( wxWindow* aParent, const wxPoint& aPos, const wxSize& aSize,
                              std::map<wxString, wxFileName> aTitleDirMap );

    /**
     * @return the selected template, or NULL
     */
    PROJECT_TEMPLATE* GetSelectedTemplate();

    void SetWidget( TEMPLATE_WIDGET* aWidget );

protected:
    void AddTemplate( int aPage, PROJECT_TEMPLATE* aTemplate );

private:
    void SetHtml( const wxFileName& aFilename )
    {
        m_htmlWin->LoadPage( aFilename.GetFullPath() );
    }

private:
    void buildPageContent( const wxString& aPath, int aPage );
    void replaceCurrentPage();

    void OnPageChange( wxNotebookEvent& event ) override;
    void onDirectoryBrowseClicked( wxCommandEvent& event ) override;
	void onReload( wxCommandEvent& event ) override;
	void OnHtmlLinkActivated( wxHtmlLinkEvent& event ) override;

protected:
    std::vector<TEMPLATE_SELECTION_PANEL*> m_panels;
    TEMPLATE_WIDGET*                       m_selectedWidget;
};

#endif
