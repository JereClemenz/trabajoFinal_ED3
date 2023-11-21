import numpy as np
import matplotlib.pyplot as plt

'''
    m = pwmMax - pwmMin/ adcMax 
    b = pwmMin - m * adcMin
'''

pwmMax = 2400
pwmMin = 500
adcMax = 255
adcMin = 0
values = np.arange(0,255,1)
m = (pwmMax - pwmMin)/ adcMax
b = pwmMin
y = m*values+b
plt.plot(values,y)
plt.grid(True)
plt.title('Mapeo PWM')
plt.xlabel('Valor ADC deseado')
plt.ylabel('Valor PWM a escribir')
plt.show()

