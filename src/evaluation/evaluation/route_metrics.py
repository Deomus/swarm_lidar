"""Геометрическая ошибка относительно пространственного маршрута без ROS."""

import math

import numpy as np


def distance_to_path(point, points):
    """Возвращает ближайшее расстояние от XYZ-точки до пространственной ломаной.

    Проекция ограничена концами каждого отрезка. Повторные вершины допустимы;
    маршрут из одной вершины считается точкой. Пустые, неконечные или неверно
    размерные входные данные вызывают ValueError. Единицы результата совпадают
    с единицами координат.
    """
    try:
        point = np.asarray(point, dtype=float)
        points = np.asarray(points, dtype=float)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError('Требуются числовая XYZ-точка и непустой массив XYZ-вершин') from error
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError('Точка должна содержать три конечные координаты')
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) == 0
            or not np.isfinite(points).all()):
        raise ValueError('Путь должен содержать непустой массив конечных XYZ-вершин')
    if len(points) == 1:
        result = math.dist(point, points[0])
    else:
        # Нормирование предотвращает переполнение квадратов больших координат
        # и потерю малых сегментов из-за возведения очень малых чисел в квадрат.
        scale = max(float(np.abs(point).max()), float(np.abs(points).max()))
        if scale == 0:
            return 0.
        normalized_point = point / scale
        normalized_points = points / scale
        starts = normalized_points[:-1]
        segments = normalized_points[1:] - starts
        squared_lengths = np.einsum('ij,ij->i', segments, segments)
        products = np.einsum('ij,ij->i', normalized_point - starts, segments)
        fractions = np.divide(products, squared_lengths,
                              out=np.zeros_like(products), where=squared_lengths > 0)
        closest = starts + np.clip(fractions, 0., 1.)[:, None] * segments
        result = float(np.linalg.norm(normalized_point - closest, axis=1).min()) * scale
    if not math.isfinite(result):
        raise ValueError('Расстояние не представимо конечным числом')
    return float(result)
