import math
rpm = int(raw_input("rpm = "))

#1st order
# For left line: intercept =  -41.791779713292314 coeff =  [0.15338737]
# For right line: intercept =  -41.86257414592657 coeff =  [0.15232979]
# pwm_1_L = -41.791779713292314 + 0.15338737*pow(rpm, 1)
pwm_1_R = 41.86257414592657 + 0.15232979*pow(rpm, 1)

#2nd order
# For left line: intercept =  -38.68910773162629 coeff =  [0.30485888 0.00120413]
# For right line: intercept = -38.66478644087105 coeff =  [0.31039519 0.00124881]
# pwm_2_L = -38.68910773162629 + 0.30485888*pow(rpm,1) + 0.00120413*pow(rpm,2)
# pwm_2_R = -38.66478644087105 + 0.31039519*pow(rpm,1) + 0.00124881*pow(rpm,2)

#3rd order
# For left line: intercept =  -37.03776982797167 coeff =  [4.34821416e-01 3.62311110e-03 1.22543795e-05]
# For right line: intercept = -36.96120656794921 coeff =  [4.47453795e-01 3.81853783e-03 1.29930905e-05]
# pwm_3_L = -37.03776982797167 + 4.34821416*pow(10,-1)*pow(rpm,1) + 3.62311110*pow(10,-3)*pow(rpm,2) + 1.22543795*pow(10,-5)*pow(rpm,3)
# pwm_3_R = -36.96120656794921 + 4.47453795*pow(10,-1)*pow(rpm,1) + 3.81853783*pow(10,-3)*pow(rpm,2) + 1.29930905*pow(10,-5)*pow(rpm,3)

#4th order
# For left line: intercept =  -36.515349477189446 coeff =  [4.97864910e-01 5.69980374e-03 3.64387299e-05 9.06586169e-08]
# For right line: intercept =  -36.48442864900595 coeff =  [5.07629067e-01 5.84474708e-03 3.67983932e-05 8.94353738e-08]
# pwm_4_L = -36.515349477189446 + 4.97864910*pow(10,-1)*pow(rpm,1) + 5.69980374*pow(10,-3)*pow(rpm,2) + 3.64387299*pow(10,-5)*pow(rpm,3) + 9.06586169*pow(10,-8)*pow(rpm,4)
# pwm_4_R = -36.48442864900595 + 5.07629067*pow(10,-1)*pow(rpm,1) + 5.84474708*pow(10,-3)*pow(rpm,2) + 3.67983932*pow(10,-5)*pow(rpm,3) + 8.94353738*pow(10,-8)*pow(rpm,4)

# -36.515349477189446
# -36.48442864900595

#5th order
# For left line: intercept =  -37.33099162225837 coeff =  [ 3.29586132e-01 -3.38701094e-03 -1.49514222e-04 -1.49430029e-06 -4.75276662e-09]
# For right line: intercept = -37.16513171110586 coeff =  [ 3.56381163e-01 -2.51085181e-03 -1.35256941e-04 -1.37652162e-06 -4.38130221e-09]
# pwm_5_L = -37.33099162225837 + 3.29586132*pow(10,-1)*pow(rpm,1) - 3.38701094*pow(10,-3)*pow(rpm,2) - 1.49514222*pow(10,-4)*pow(rpm,3) - 1.49430029*pow(10,-6)*pow(rpm,4) - 4.75276662*pow(10,-9)*pow(rpm,5)
# pwm_5_R = -37.16513171110586 + 3.56381163*pow(10,-1)*pow(rpm,1) - 2.51085181*pow(10,-3)*pow(rpm,2) - 1.35256941*pow(10,-4)*pow(rpm,3) - 1.37652162*pow(10,-6)*pow(rpm,4) - 4.38130221*pow(10,-9)*pow(rpm,5)

#6th order
# For left line: intercept =  -37.87714357443103 coeff =  [ 7.99788666e-02 -2.41136679e-02 -7.98765555e-04 -1.07693221e-05 -6.59511368e-08 -1.51331211e-10]
# For right line: intercept = -37.65586473022285 coeff =  [ 1.12257923e-01 -2.30063537e-02 -7.77725184e-04 -1.05254248e-05 -6.44333008e-08 -1.47558188e-10]
# pwm_6_L = -37.87714357443103 + 7.99788666*pow(10,-2)*pow(rpm,1) - 2.41136679*pow(10,-2)*pow(rpm,2) - 7.98765555*pow(10,-4)*pow(rpm,3) - 1.07693221*pow(10,-5)*pow(rpm,4) - 6.59511368*pow(10,-8)*pow(rpm,5) - 1.51331211*pow(10,-10)*pow(rpm,6)
# pwm_6_L = -37.65586473022285 + 1.12257923*pow(10,-1)*pow(rpm,1) - 2.30063537*pow(10,-2)*pow(rpm,2) - 7.77725184*pow(10,-4)*pow(rpm,3) - 1.05254248*pow(10,-5)*pow(rpm,4) - 6.44333008*pow(10,-8)*pow(rpm,5) - 1.47558188*pow(10,-10)*pow(rpm,6)

# print "pwm_1_L = ", pwm_1_L
print "pwm_1_R = ", pwm_1_R
# print "pwm_2_L = ", pwm_2_L, "\tpwm_2_R = ", pwm_2_R
# print "pwm_3_L = ", pwm_3_L, "\tpwm_3_R = ", pwm_3_R
# print "pwm_4_L = ", pwm_4_L, "\tpwm_4_R = ", pwm_4_R
# print "pwm_5_L = ", pwm_5_L, "\tpwm_5_R = ", pwm_5_R