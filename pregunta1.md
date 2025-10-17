2.1

Segun la documentacion de la libreria scipy:
- Sirve para resolver ecuaciones diferenciales ordinarias usando lsoda de la libreria FORTRAN odepack.
- Resuelve al valor inicial de problemas rigidos y no rigidos, en sistemas de primer orden ode-s.
https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.odeint.html

odeint resuelve ecuaciones diferenciales de forma numerica, donde la entrada puede ser un vector, internamente usa un metodo lsoda, que cambia de metodo dependiendo si el problema es rigido o no rigido. A la funcion se le debe entregar la EDO, con las condiciones iniciales y vectores de tiempo, el resultado de la funcion es una matriz de valores, donde cada fila corresponde a una solucion para el tiempo determinado.
