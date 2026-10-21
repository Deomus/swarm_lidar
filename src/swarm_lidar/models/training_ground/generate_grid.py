#!/usr/bin/env python3
"""Воспроизводимая плоская разметка одним COLLADA mesh, без внешних ресурсов."""

from collections import defaultdict
from pathlib import Path
import xml.etree.ElementTree as ET


COLORS = {
    'background': (.58, .62, .66, 1.),
    'grid': (.90, .93, .96, 1.),
    'x_axis': (.95, .22, .12, 1.),
    'y_axis': (.10, .38, .95, 1.),
}


def generate(path):
    vertices, indices, triangles = [], {}, defaultdict(list)
    def vertex(point):
        point = tuple(round(value, 6) for value in point)
        if point not in indices:
            indices[point] = len(vertices)
            vertices.append(point)
        return indices[point]

    def rectangle(material, x0, y0, x1, y1, z=.002):
        corners = [vertex(point) for point in
                   [(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]]
        triangles[material].extend([corners[:3], [corners[0], corners[2], corners[3]]])

    extent, axis_half = 100., .09
    rectangle('background', -extent, -extent, extent, extent, .001)
    for coordinate in range(-100, 101, 2):
        if coordinate == 0:
            continue
        half = .055 if coordinate % 10 == 0 else .0175
        lo, hi = max(-extent, coordinate-half), min(extent, coordinate+half)
        # Обычные линии не накрывают разноцветные оси. На пересечениях
        # остальных линий одинаковы высота, нормаль и материал.
        rectangle('grid', -extent, lo, -axis_half, hi)
        rectangle('grid', axis_half, lo, extent, hi)
        rectangle('grid', lo, -extent, hi, -axis_half)
        rectangle('grid', lo, axis_half, hi, extent)
    rectangle('x_axis', -extent, -axis_half, extent, axis_half)
    rectangle('y_axis', -axis_half, -extent, axis_half, -axis_half)
    rectangle('y_axis', -axis_half, axis_half, axis_half, extent)

    root = ET.Element('COLLADA', xmlns='http://www.collada.org/2005/11/COLLADASchema', version='1.4.1')
    asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'created').text = '2026-10-21T00:00:00Z'
    ET.SubElement(asset, 'modified').text = '2026-10-21T00:00:00Z'
    ET.SubElement(asset, 'unit', name='meter', meter='1')
    ET.SubElement(asset, 'up_axis').text = 'Z_UP'
    effects, materials = ET.SubElement(root, 'library_effects'), ET.SubElement(root, 'library_materials')
    for name, color in COLORS.items():
        effect = ET.SubElement(effects, 'effect', id=f'{name}_effect')
        profile = ET.SubElement(effect, 'profile_COMMON')
        technique = ET.SubElement(profile, 'technique', sid='common')
        shader = ET.SubElement(technique, 'lambert')
        for component in ('emission', 'ambient', 'diffuse'):
            values = (*[v*.15 for v in color[:3]], 1.) if component == 'emission' else color
            ET.SubElement(ET.SubElement(shader, component), 'color').text = ' '.join(map(str, values))
        material = ET.SubElement(materials, 'material', id=f'{name}_material', name=name)
        ET.SubElement(material, 'instance_effect', url=f'#{name}_effect')
    geometry = ET.SubElement(ET.SubElement(root, 'library_geometries'), 'geometry', id='ground_grid')
    mesh = ET.SubElement(geometry, 'mesh')
    for name, points, parameters in [('positions', vertices, ('X', 'Y', 'Z')),
                                     ('normals', [(0., 0., 1.)], ('X', 'Y', 'Z'))]:
        source = ET.SubElement(mesh, 'source', id=name)
        ET.SubElement(source, 'float_array', id=f'{name}_array', count=str(len(points)*3)).text = ' '.join(
            format(value, '.6f') for point in points for value in point)
        accessor = ET.SubElement(ET.SubElement(source, 'technique_common'), 'accessor',
                                 source=f'#{name}_array', count=str(len(points)), stride='3')
        for parameter in parameters:
            ET.SubElement(accessor, 'param', name=parameter, type='float')
    vertex_input = ET.SubElement(mesh, 'vertices', id='ground_vertices')
    ET.SubElement(vertex_input, 'input', semantic='POSITION', source='#positions')
    for name, faces in triangles.items():
        group = ET.SubElement(mesh, 'triangles', material=name, count=str(len(faces)))
        ET.SubElement(group, 'input', semantic='VERTEX', source='#ground_vertices', offset='0')
        ET.SubElement(group, 'input', semantic='NORMAL', source='#normals', offset='1')
        ET.SubElement(group, 'p').text = ' '.join(f'{index} 0' for face in faces for index in face)
    scene = ET.SubElement(ET.SubElement(root, 'library_visual_scenes'), 'visual_scene', id='ground_scene')
    instance = ET.SubElement(ET.SubElement(scene, 'node', id='ground_node'), 'instance_geometry', url='#ground_grid')
    bindings = ET.SubElement(ET.SubElement(instance, 'bind_material'), 'technique_common')
    for name in COLORS:
        ET.SubElement(bindings, 'instance_material', symbol=name, target=f'#{name}_material')
    ET.SubElement(ET.SubElement(root, 'scene'), 'instance_visual_scene', url='#ground_scene')
    ET.indent(root, space='  ')
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)
    print(f'{path}: {len(vertices)} вершин, {sum(map(len, triangles.values()))} треугольников')


if __name__ == '__main__':
    generate(Path(__file__).parent/'meshes/grid.dae')
