import numpy as np
import matplotlib.pyplot as plt

'''
    m = pwmMax - pwmMin/ adcMax 
    b = pwmMin - m * adcMin
'''

volts = np.arange(0,3.3,(3.3/255))
adcMax = 255
adcMin = 0
values = np.arange(0,255,1)
m = (adcMax-adcMin)/3.3
b = 0
y = m*volts+b
plt.plot(volts,y)
plt.grid(True)
plt.title('Mapeo Voltaje')
plt.xlabel('Valor voltaje [V]')
plt.ylabel('Valor ADC deseado')
plt.show()
