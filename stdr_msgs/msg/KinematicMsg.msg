# Container for the kinematic model parameters. The parameters selected/
# are quite general. For a more accurate motion model a per-kinematic model
# approach should be followed.
# Parameters are in the form a_C_D, where C is affected by D.
# ux is the linear speed
# uy is the lateral speed (for omni vehicles)
# w is the angular speed
# g is a cofficient that directly affects the angular speed
# For more information check the MotionController::sampleVelocities function.

string type
float32 a_ux_ux
float32 a_ux_uy
float32 a_ux_w
float32 a_uy_ux
float32 a_uy_uy
float32 a_uy_w
float32 a_w_ux
float32 a_w_uy
float32 a_w_w
float32 a_g_ux
float32 a_g_uy
float32 a_g_w
