# План строя от ведущего ведомым по mesh. Ведомый сам вычисляет свои места
# на момент t: переход start_offsets → end_offsets по сглаженному шагу
# s(u) = u²(3 − 2u), u = (t − start_time) / duration. Места — в системе
# курса ведущего (FLU), строка i — ведомый uav_(i+1).
builtin_interfaces/Time stamp
string formation
float32 scale
# Строй включён; epoch растёт при каждом включении: ведомые сбрасывают
# регуляторы и снимают остановку по сближению прошлого включения.
bool enabled
uint32 epoch
# Время симуляции, с.
float64 start_time
float64 duration
geometry_msgs/Point[] start_offsets
geometry_msgs/Point[] end_offsets
