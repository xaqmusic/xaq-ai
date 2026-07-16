import sys, time
from PyQt6.QtWidgets import QApplication
from xaq.server.lateral_voter import LateralVoterNode
from xaq.server.ui.widgets.timeline_widget import TimelineWidget

app = QApplication(sys.argv)
voter = LateralVoterNode()

widget = TimelineWidget(voter)
widget.show()

# Setup some tokens
class FakeToken:
    def __init__(self, source_id, active_node_id, timestamp):
        self.source_id = source_id
        self.active_node_id = active_node_id
        self.timestamp = timestamp
        self.dopamine_level = 0.5
        self.serotonin_level = 0.5

# Populate tracks
token1 = FakeToken("agent1", 5, time.time() - 1.0)
token2 = FakeToken("agent2", 10, time.time() - 1.0)

# Simulate consensus and set weight
voter.association_matrix[5][10] = 0.5
voter.association_matrix[10][5] = 0.5

widget.update_timeline({'candidates': [token1, token2]})

import threading
def update_timer():
    while True:
        time.sleep(0.04)
        t = time.time()
        widget.update_timeline({'candidates': [
            FakeToken("agent1", 5, t),
            FakeToken("agent2", 10, t)
        ]})

threading.Thread(target=update_timer, daemon=True).start()

# Let's map coordinates and see if x_final is sane.
# We'll use a timer to run an event and exit.
def exit_app():
    # Let's check internal state
    parent = widget.container
    t1 = widget.tracks['agent1']
    t2 = widget.tracks['agent2']
    c1 = t1.timeline
    c1_origin = c1.mapTo(parent, c1.rect().topLeft())
    print("c1 origin in container:", c1_origin)
    print("Weight:", voter.association_matrix[5][10])
    app.quit()

from PyQt6.QtCore import QTimer
QTimer.singleShot(2000, exit_app)

app.exec()
