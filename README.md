# Laboratorio 4 de Robótica
## Universiad Nacional de Colombia
## 2022-2
***
### Autores
- Andrés Holguín Restrepo 
- Julián Andrés Caipa Prieto
### Profesores encargados
- Ing. Ricardo Emiro Ramírez H.
- Ing. Jhoan Sebastian Rodriguez R.
***
### Introducción

### Cinemática directa


#### Mediciones del pincher

#### Tabla DHstd

#### ToolBox



### Desarrollo código python



#### Datos iniciales

AHora bien, lo primero que se debe establecer es que hay ciertas limitantes que se dieron. Debido a que en código uno lo que puede realizar es cambiar los valores de registro del pincher, en este caso de 0 a 1023 para los registros de posición que van de un rango de 0° a 300°, es necesario conocer los valores puntuales de los registros para obtener estas posiciones. Esto se logra mediante el uso del Dynamixel Wizard. AL llevar el Picher a la posición final que queremos, se pueden extraer los valores de los registros directamente y almacenarlos para su implementación futura. Dicho esto, a continuación se muestran los valores de los registros teniendo en cuenta las poses deseadas y el Home previamente establecido.

#### Python
Para el desarrollo de esta práctica se decidió trabajar con python en vez de Matlab ya que se considera de este modo más facil la configuración.

| Pose     | Registro M1 |Registro M2 |Registro M3 |Registro M4 |Registro M5 |
| :----:   | :----:      | :----:     | :----:     | :----:     | :----:     |
|home      |514          |510         |818         |512         |512         |
|1         |444          |575         |751         |580         |512         |
|2         |614          |410         |922         |409         |512         |
|3         |205          |560         |630         |570         |512         |
|4         |205          |660         |629         |666         |546         |



### Resultados


 
 
 
