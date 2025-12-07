void EncoderInit();
void EncoderReset();
void Forward_Degree(int left_power,int right_power,int degree);
void Reverse_Degree(int left_power,int right_power,int degree);
extern double error,PID_out;
void PID_Turn(int degree);
void PID_Move(int left_power,int right_power,int degree,int timelimit);