from PyQt6.QtWidgets import QTableWidget, QTableWidgetItem, QHeaderView
from PyQt6.QtGui import QColor

class EPMListWidget(QTableWidget):
    def __init__(self):
        super().__init__()
        self.setColumnCount(5)
        self.setHorizontalHeaderLabels([
            "Agent ID", "State", "Latency", "Dopamine", "Serotonin"
        ])
        self.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        self.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        
        # Keep track of rows: agent_id -> row_index
        self.agent_rows = {}

    def update_agent(self, agent_id, state_data):
        """
        Update or add an agent to the list.
        stat_data expected to have keys matching our columns.
        """
        if agent_id not in self.agent_rows:
            self.insertRow(self.rowCount())
            row = self.rowCount() - 1
            self.agent_rows[agent_id] = row
            
            # Init Items
            for col in range(5):
                self.setItem(row, col, QTableWidgetItem())
        
        row = self.agent_rows[agent_id]
        
        # Populate Data
        # agent_id
        self.item(row, 0).setText(agent_id)
        
        # State
        state = state_data.get('state', 'UNKNOWN')
        item_state = self.item(row, 1)
        item_state.setText(state)
        
        # Color Code
        if state == "RUNNING":
            item_state.setForeground(QColor("green"))
        elif state == "BOOTSTRAPPING":
            item_state.setForeground(QColor("cyan"))
        elif state == "PAUSED":
            item_state.setForeground(QColor("gray"))
        else:
            item_state.setForeground(QColor("red"))
            
        # Metrics
        self.item(row, 2).setText(state_data.get('latency', '---'))
        self.item(row, 3).setText(f"{state_data.get('dopamine', 0.0):.2f}")
        self.item(row, 4).setText(f"{state_data.get('serotonin', 0.0):.2f}")

    def remove_agent(self, agent_id):
        if agent_id in self.agent_rows:
            row = self.agent_rows[agent_id]
            self.removeRow(row)
            del self.agent_rows[agent_id]
            # Rebuild index map because rows shifted
            self._rebuild_row_map()

    def _rebuild_row_map(self):
        self.agent_rows = {}
        for r in range(self.rowCount()):
            aid = self.item(r, 0).text()
            self.agent_rows[aid] = r
