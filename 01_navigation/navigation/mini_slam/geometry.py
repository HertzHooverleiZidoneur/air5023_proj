"""Published nominal AIR5023 differential-drive geometry in metres.

Defaults are transcribed from the public Google Robot MJCF (wheel radius and
wheel-center offsets) and the team experiment contract; applications can
override them when constructing a custom estimator. Source:
https://github.com/google-deepmind/mujoco_menagerie/blob/main/google_robot/robot.xml#L105-L111
"""

WHEEL_RADIUS = 0.06435
TRACK = 0.36566
