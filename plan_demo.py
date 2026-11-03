"""Try the tag tour planner on its own (no ROS, no robot).

  python3 plan_demo.py                                   # the 4 simulation tags
  python3 plan_demo.py --tags "2,1 -1.5,2 1,-2.5 -2,-1" --start 0,0,0
  python3 plan_demo.py --tags "1,0 2,0 2,1 0,2 -1,1" --w 0.45 --plot plan.png

Prints the best order (Held-Karp), every leg (turn, drive, time), and compares it
with nearest-neighbour and, for up to 8 tags, with all possible orders.
"""

import argparse
import itertools
import math

from tour_planner import MotionModel, evaluate, plan_tour


def parse_points(text):
    return [tuple(float(v) for v in p.split(',')) for p in text.split()]


def nearest_neighbour(start, tags):
    order, here, left = [], start, set(range(len(tags)))
    while left:
        k = min(left, key=lambda i: math.dist(here, tags[i]))
        order.append(k)
        left.remove(k)
        here = tags[k]
    return order


def plot(path, start, heading, tags, tour, nn_tour, model):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 7), dpi=110)
    for t, style, color, label in ((nn_tour, '--', '#c2185b', 'nearest neighbour'),
                                   (tour, '-', '#1e88e5', 'planner (time-optimal)')):
        pts = [start] + [leg.stop for leg in t.legs]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], style, color=color, lw=2.2,
                label=f'{label}: {t.time:.1f} s, {t.distance:.2f} m')
    for n, leg in enumerate(tour.legs, 1):
        x, y = tags[leg.tag]
        ax.scatter([x], [y], marker='s', s=140, color='#2e7d32', edgecolor='black', zorder=5)
        ax.add_patch(plt.Circle((x, y), model.standoff, fill=False, ls=':', color='#2e7d32'))
        ax.annotate(f'tag {leg.tag}  #{n}', (x, y), xytext=(8, 8), textcoords='offset points',
                    weight='bold', color='#2e7d32')
    ax.scatter([start[0]], [start[1]], s=90, color='black', zorder=6, label='start')
    ax.arrow(start[0], start[1], 0.3 * math.cos(heading), 0.3 * math.sin(heading),
             width=0.02, color='black', length_includes_head=True, zorder=6)
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(f'Tag tour plan (v_max {model.v_max} m/s, w_max {model.w_max} rad/s)')
    ax.legend(loc='best', fontsize=8)
    fig.tight_layout()
    fig.savefig(path)
    print(f'\nplot saved: {path}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--tags', default='2,1 -1.5,2 1,-2.5 -2,-1', help='"x,y x,y ..." in metres')
    ap.add_argument('--start', default='0,0,0', help='x,y,heading_deg of the robot')
    ap.add_argument('--v', type=float, default=0.25, help='v_max, m/s')
    ap.add_argument('--w', type=float, default=0.45, help='w_max, rad/s (real turn rate)')
    ap.add_argument('--standoff', type=float, default=0.5, help='stop this far before a tag, m')
    ap.add_argument('--overhead', type=float, default=0.5, help='seconds per leg')
    ap.add_argument('--exact-limit', type=int, default=10, help='Held-Karp up to this many tags')
    ap.add_argument('--plot', default='plan.png', help='PNG file ("" = no plot)')
    a = ap.parse_args()

    tags = parse_points(a.tags)
    sx, sy, sh = (float(v) for v in a.start.split(','))
    start, heading = (sx, sy), math.radians(sh)
    model = MotionModel(a.v, a.w, a.standoff, a.overhead)

    tour = plan_tour(start, heading, tags, model, a.exact_limit)
    print(f'{len(tags)} tags, start ({sx:+.2f}, {sy:+.2f}) facing {sh:.0f} deg, method: {tour.method}')
    print(f'best order: {tour.order}  ->  {tour.time:.1f} s, {tour.distance:.2f} m\n')
    print(' #  tag   turn     drive    time')
    for n, leg in enumerate(tour.legs, 1):
        print(f'{n:2d}  {leg.tag:3d}  {math.degrees(leg.turn):+5.0f} deg  {leg.distance:5.2f} m  '
              f'{leg.time:5.1f} s')

    nn = evaluate(nearest_neighbour(start, tags), start, heading, tags, model)
    print(f'\nnearest neighbour: {nn.order}  ->  {nn.time:.1f} s, {nn.distance:.2f} m '
          f'({nn.time - tour.time:+.1f} s vs planner)')
    if len(tags) <= 8:
        times = sorted((evaluate(list(o), start, heading, tags, model).time, o)
                       for o in itertools.permutations(range(len(tags))))
        print(f'all {len(times)} orders: best {times[0][0]:.1f} s {list(times[0][1])}, '
              f'worst {times[-1][0]:.1f} s {list(times[-1][1])}')
    if a.plot:
        plot(a.plot, start, heading, tags, tour, nn, model)


if __name__ == '__main__':
    main()
