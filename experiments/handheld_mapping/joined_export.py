"""Export gravity-preserving planar ground with separate observed obstacle surfaces."""
import argparse
import json
from pathlib import Path


def export_run(run, output, backend='auto', depth_max=None):
    from .viewer import select_maps
    from .depth_variants import ensure_depth_variant
    from .surface_colors import load_surface_layers
    from .planar_terrain import build
    run, output = Path(run).resolve(), Path(output).resolve()
    if depth_max is not None:
        run = ensure_depth_variant(run, depth_max, backend)
    maps = select_maps(run, backend=backend)
    if any(load_surface_layers(path) is None for _, path in maps):
        if (run/'depth_variant.json').is_file() or (run/'merge.json').is_file():
            raise ValueError('이 지도의 지면·장애물 분류가 없습니다. 중력 기준과 지형 생성 결과를 확인하세요.')
        run = ensure_depth_variant(run, depth_max or 3., backend)
        maps = select_maps(run, backend=backend)
    output.mkdir(parents=True, exist_ok=False)
    results = []
    for name, path in maps:
        folder = output / name.replace(' ', '_')
        result = build(path, folder/'planar')
        results.append(result)
    (output/'summary.json').write_text(json.dumps({'status':'complete', 'maps':results}, indent=2))
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--backend', default='auto')
    parser.add_argument('--depth-max', type=float)
    args = parser.parse_args()
    export_run(args.run, args.output, args.backend, args.depth_max)


if __name__ == '__main__':
    main()
