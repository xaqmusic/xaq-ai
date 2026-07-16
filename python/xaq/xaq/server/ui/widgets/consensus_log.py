from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QTableWidget, QTableWidgetItem, QHeaderView
from collections import Counter

class ConsensusLogWidget(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        
        self.total_label = QLabel("Active Nodes: 0")
        self.total_label.setStyleSheet("font-weight: bold; font-size: 14px; color: #00FF00;")
        layout.addWidget(self.total_label)
        
        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["Node IDs", "Text Label", "Trigger Count"])
        
        # Adjust column sizes
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        
        layout.addWidget(self.table)
        
    def update_log(self, voter_node, node_count=0):
        self.total_label.setText(f"Active Nodes: {node_count}")
        
        # Get top 10 unique combinations directly from the persistent counter
        # The counter holds up to 10k unique tuples over the entire server uptime
        # Note: most_common internally truncates safely to the top 10.
        top_10 = voter_node.concept_counts.most_common(10)
        
        self.table.setRowCount(len(top_10))
        for row, (group_key, count) in enumerate(top_10):
            # IDs
            id_str = ", ".join(str(i) for i in group_key)
            self.table.setItem(row, 0, QTableWidgetItem(id_str))
            
            # Label
            label_str = voter_node.concept_labels.get(group_key, "")
            self.table.setItem(row, 1, QTableWidgetItem(label_str))
            
            # Count
            self.table.setItem(row, 2, QTableWidgetItem(str(count)))
