string state
string message
uint32 park_attempt   # parking spots suggested this pick
bool grabbed          # pass fail: the jaw closed on something (real), or the handle sat between the open blades (jaw left open)
float64 miss_m        # grasp point to the handle's bar when judged by eye, the tracker score; NaN when the jaw reading judged
