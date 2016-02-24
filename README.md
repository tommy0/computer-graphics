# computer-graphics
implementation of low-level algorithms of computer graphics

Реализация низкоуровневых алгоритмов компьютерной графики (изночально умеем только ставить точку):
1)определение точки относительно прямой
2)определение точки и типа пересечния прямых
3)определение поподание точки в полигон двумя способами(NZW и EO)
4)определение типа полигона(сложный, не сложный, выпуклый, не выпуклый)
5)отрисовка линии методом Брезенхэма
6)отрисовка полигона
7)заполнение полигона
8)отрисовка окружности методом Брезенхэма
9)отрисовка линий разной толщины (с разными оканцовками: квадратной, закругленной, без них)
10)отрисовка пунктирных линий задаваемых собственным стилем
11)отрисовка составных B сплайнивых кривых
12)отсечение выпуклого полигона выпуклым полигоном
13) отсечение прямой полигоном
14)отрисовка кривых Бизье 3 порядка
15)отрисовка кривых Бизье N порядка
16)работа с 3D координатами
17)отрисовка простого кубика(храним списком ребер)
18)поворот кубика
19)паралельный перенос кубика
20)паралельная проекция
21)перспективная проекция
22)удаление нелицевых граней(редринг)
23)реализация точечного источника света
24)Заполение цветом кубика методом Гуро
25)Отрисовка B сплайновой кривой методом плавающего горизонта
*вместо кубика возможна любая другая выпуклая 3D фигура
наглядные тесты для демонстрации прилагаются в коментариях проекта
gfSetPixel() - ставит точку в выбраные координаты