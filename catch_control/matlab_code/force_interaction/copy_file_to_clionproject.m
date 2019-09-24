
source_path = '/home/gx/Projects/match/catch_control/matlab_code/force_interaction/force_interaction_ert_rtw';
target_path = '/home/gx/Projects/match/catch_control/simulink2C/force_interaction';




copyfile ([source_path '/force_interaction.h'],target_path)
copyfile ([source_path '/force_interaction.cpp'],target_path)
copyfile ([source_path '/rtwtypes.h'],target_path)



