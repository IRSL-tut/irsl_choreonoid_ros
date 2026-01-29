# ---
# jupyter:
#   jupytext:
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.18.1
#   kernelspec:
#     display_name: Choreonoid
#     language: python
#     name: choreonoid
# ---
# jupytext --to ipynb -o output.ipynb ros_debug.py

# %%
import rospy
from IPython.display import display
import ipywidgets as widgets
from threading import Lock

# %%
out = widgets.Output()
display(out)

# %%
lock = Lock()
def callback(event):
    rospy.loginfo(f'info: {event.current_real}') ## see /rosout
    with lock:
        out.append_stdout('out: ' + str(event.current_real) + '\n')


# %%
rospy.init_node('timer_test')

# %%
rospy.Timer(rospy.Duration(1), callback)
