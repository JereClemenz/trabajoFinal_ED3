import numpy as np
import matplotlib.pyplot as plt

# Supongamos que tienes una serie de valores, por ejemplo:
datos = np.array([2724,2728,2727,2728,2723,2725,2723,2728,2725,2725,2725,2723,
                  2727,2729,2727,2727,2723,2725,2728,2729,2728,2729,2725,2728,
                  2729,2728,2724,2728,2724,2728,2729,2725,2728,2725,2727,2724,
                  2728,2724,2725,2727,2727,2729,2727,2725,2727,2724,2723,2728,
                  2727,2725
                  ])

# Calcular la media
media = np.mean(datos)

# Calcular el error (suponiendo que tienes una medida de error específica, de lo contrario, puedes omitir esto)
error = np.std(datos)  # Puedes cambiar esto según tu medida de error

# Calcular la desviación estándar
desviacion_estandar = np.std(datos)

# Graficar la variación de los datos
plt.plot(datos, marker='o', linestyle='-', color='b', label='Datos')
plt.axhline(media, color='r', linestyle='--', label='Media')
plt.fill_between(range(len(datos)), media - error, media + error, color='r', alpha=0.2, label='Error')
plt.xlabel('Índice de datos')
plt.ylabel('Valor')
plt.legend()
plt.show()

# Imprimir los resultados
print(f'Media: {media}')
print(f'Error: {error}')
print(f'Desviación Estándar: {desviacion_estandar}')