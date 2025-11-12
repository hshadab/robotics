# Archive Directory

This directory contains deprecated or legacy code that is no longer actively used but kept for reference.

## Contents

### legacy-tkinter-ui/
**Deprecated**: Old Tkinter-based UI (`demo_ui.py`)

- **Replaced by**: Web-based UI in `tools/robotics-ui/`
- **Status**: No longer maintained
- **Reason**: Modern web UI provides better demo experience with real-time updates
- **When it was used**: Early prototype phase
- **Safe to delete**: Yes, if you don't need the reference implementation

## Migration Notes

If you were using the old Tkinter UI:
1. Use `tools/robotics-ui/server.js` instead (Node.js web server)
2. Access the demo at `http://localhost:9200` in your browser
3. All functionality has been migrated to the web UI

## Restoring Legacy Code

To restore any archived component:
```bash
# Example: restore legacy UI
mv archive/legacy-tkinter-ui tools/ui
```

Note: Restored code may have unresolved dependencies or compatibility issues.
