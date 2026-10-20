"""Inspect a fixed slope layout with its orange inferred ramp-step faces."""
import argparse
import json
import os
from pathlib import Path

from .slope_step import sha256


def main(layout_dir=None):
    parser = argparse.ArgumentParser(description=__doc__)
    if layout_dir is None:
        parser.add_argument('layout_dir', type=Path)
    parser.add_argument('--render-only', action='store_true')
    args = parser.parse_args()
    root = Path(layout_dir if layout_dir is not None else args.layout_dir).resolve()
    report = json.loads((root/'ramp_steps.json').read_text())
    if report['alignment_sha256'] != sha256(root/'alignment.json'):
        raise ValueError('layout changed; regenerate ramp steps')
    for row in report['layouts']:
        for name, expected in row['input_sha256'].items():
            if sha256(root/name) != expected:
                raise ValueError('source display mesh changed; regenerate ramp steps')
    os.environ.pop('WAYLAND_DISPLAY', None)
    os.environ['XDG_SESSION_TYPE'] = 'x11'
    import open3d as o3d
    viewer = o3d.visualization.VisualizerWithKeyCallback()
    if not viewer.create_window(
        window_name='Slope layout | R: smooth ramps | S: step on/off | F: full context | 1/2/3: layout',
        width=1280, height=900, visible=not args.render_only):
        raise RuntimeError('3D display unavailable')
    options = viewer.get_render_option()
    options.background_color = [.055, .07, .10]
    options.mesh_show_back_face = True
    options.mesh_color_option = o3d.visualization.MeshColorOption.Color
    current = [None]; index = [0]; full = [False]; steps = [True]; cache = {}
    smooth = [True]

    def select(which, preserve=True):
        index[0] = which
        camera = viewer.get_view_control().convert_to_pinhole_camera_parameters() if preserve else None
        if current[0] is not None:
            viewer.remove_geometry(current[0], reset_bounding_box=False)
        suffix = 'full_context' if full[0] else 'mesh'
        corrected = smooth[0] and steps[0] and 'ramp_polish' in report['layouts'][which]
        name = f'hypothesis_{which+1}_{suffix}'+('_polished_with_step' if corrected else
                                                       '_with_step' if steps[0] else '')+'.ply'
        if name not in cache:
            cache[name] = o3d.io.read_triangle_mesh(str(root/name))
            if not len(cache[name].triangles):
                raise ValueError('missing slope layout: '+name)
            cache[name].compute_vertex_normals()
        current[0] = cache[name]
        viewer.add_geometry(current[0], reset_bounding_box=not preserve)
        if preserve:
            viewer.get_view_control().convert_from_pinhole_camera_parameters(camera, allow_arbitrary=True)
        print(f'LAYOUT {which+1} | inferred ramp step: {steps[0]} | smooth ramps: {corrected}', flush=True)
        return False

    def toggle(state):
        state[0] = not state[0]
        return select(index[0])

    for which in range(len(report['layouts'])):
        viewer.register_key_callback(ord(str(which+1)), lambda v, i=which: select(i))
    viewer.register_key_callback(ord('S'), lambda v: toggle(steps))
    viewer.register_key_callback(ord('F'), lambda v: toggle(full))
    viewer.register_key_callback(ord('R'), lambda v: toggle(smooth))
    select(0, preserve=False)
    control = viewer.get_view_control()
    control.set_up([0, 0, 1]); control.set_front([1, -1, .8])
    control.set_lookat(current[0].get_center()); control.set_zoom(.65)
    if args.render_only:
        for which in range(len(report['layouts'])):
            select(which)
            for _ in range(8):
                viewer.poll_events(); viewer.update_renderer()
            viewer.capture_screen_image(str(root/f'hypothesis_{which+1}_polished_step_preview.png'), do_render=True)
    else:
        viewer.run()
    viewer.destroy_window()


if __name__ == '__main__':
    main()
