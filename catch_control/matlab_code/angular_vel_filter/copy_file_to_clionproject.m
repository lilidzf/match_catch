
source_path = '/home/zd/project/catch_bottle/catch_control/matlab_code/angular_vel_filter/end_point_filter_ert_rtw';
target_path = '/home/zd/project/catch_bottle/catch_control/simulink2C/end_point_filter';
copyfile ([source_path '/end_point_filter.h'],target_path)
copyfile ([source_path '/end_point_filter.cpp'],target_path)
copyfile ([source_path '/rtwtypes.h'],target_path)



