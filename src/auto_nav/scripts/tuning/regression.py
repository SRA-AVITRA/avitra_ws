import numpy as np 
import matplotlib.pyplot as plt 
import pandas as pd 
from sklearn.linear_model import LinearRegression 
from sklearn.preprocessing import PolynomialFeatures 

datas = pd.read_csv('/home/swapnil/avitra_ws/src/auto_nav/observations_for_analysis/sheets/csv/rpm_vs_pwm_a_buck_23V.csv')

fig, axs = plt.subplots(2)

# rpm_L = datas.iloc[0:12998, 0:1].values
# pwm_L = datas.iloc[0:12998, 1].values 
# rpm_R = datas.iloc[0:15818, 3:4].values
# pwm_R = datas.iloc[0:15818, 4].values

rpm_L = datas.iloc[0:12998, 6:7].values
pwm_L = datas.iloc[0:12998, 7].values 
rpm_R = datas.iloc[0:15818, 9:10].values
pwm_R = datas.iloc[0:15818, 10].values

poly = PolynomialFeatures(degree = 4) 
left_poly = poly.fit_transform(rpm_L) 
right_poly = poly.fit_transform(rpm_R) 

poly.fit(left_poly, rpm_L)
poly.fit(right_poly, rpm_R) 

left_line = LinearRegression()
right_line = LinearRegression() 
left_line.fit(left_poly, pwm_L)
right_line.fit(right_poly, pwm_R)

axs[0].scatter(rpm_L, pwm_L, color = 'blue') 
axs[1].scatter(rpm_R, pwm_R, color = 'blue') 

axs[0].plot(rpm_L, left_line.predict(left_poly), color = 'red') 
axs[1].plot(rpm_R, right_line.predict(right_poly), color = 'red') 

axs[0].set_title('Left_motor_rpm_vs_pwm') 
axs[0].set_title('Right_motor_rpm_vs_pwm')

axs[0].set(xlabel='rpm_L', ylabel='pwm_L')
axs[1].set(xlabel='rpm_R', ylabel='pwm_R') 

print "for 4rd order positive relation:"
print "For left line: intercept = ", left_line.intercept_, "coeff = ", left_line.coef_[1:]
print "For right line: intercept = ", right_line.intercept_, "coeff = ", right_line.coef_[1:]

plt.savefig('/home/swapnil/avitra_ws/src/auto_nav/observations_for_analysis/plots/rpm_vs_pwm_buck_23V/positive_rpm_vs_pwm_a4_buck_23V.png')
plt.show()
