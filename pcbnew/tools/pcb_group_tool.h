#include <tool/group_tool.h>
#include <board_commit.h>

class PCB_GROUP_TOOL : public GROUP_TOOL
{
public:
    /**
     * Invoke the picker tool to select a new member of the group.
     */
    int PickNewMember( const TOOL_EVENT& aEvent ) override;

    ///< Group selected items.
    int Group( const TOOL_EVENT& aEvent ) override;

protected:
    std::unique_ptr<COMMIT> createCommit() override { return std::make_unique<BOARD_COMMIT>( this ); }
};
