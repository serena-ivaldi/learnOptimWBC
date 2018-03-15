%% in this file i perform few operation on the data collected during the measure operation before closing the simulink scheme

 
first_value_joints = first_joints.Data(1,:);
first_value_joints_deg = first_value_joints * (180/pi);


first_value_xcom = first_com.Data(1,:);

all_q_matrix       = all_q.Data;
all_q_matrix_deg   = all_q_matrix *(180/pi);
all_Xcom_matrx     = all_Xcom.Data;

