from gworld import *
from visualize import *
import astar
import operator


def check_crossings(paths):
    max_len = 0
    for path in paths:
        if len(path) > max_len:
            max_len = len(path)

    print('Paths before: ', paths)

    print(max_len)
    i = 0
    while i < max_len:  # Iterate through the time
        # Compare positions in the specific time
        k = 0
        while k < len(paths):
            if len(paths[k]) <= i:
                k += 1
                continue
            j = k + 1
            while j < len(paths):
                if len(paths[j]) <= i:
                    j += 1
                    continue
                print(i, k, j)
                if not k == j:  # Check that we don't look at the same paths
                    print(paths[k][i], ' ', paths[j][i])
                    if paths[k][i] == paths[j][i]:  # Check collision
                        # We don't need check for i not eq 0 bcs we assume that all
                        # agents was placed in different coordinates as first point
                        paths[k].insert(i, paths[k][i-1])
                        # TODO: recalculate max_len
                        # TODO: Check obstacles (rocks)
                        print('Crossing between agents.')
                        i -= 1
                        continue
                    elif (i+1 < len(paths[k])) & (i+1 < len(paths[j])):  # Check that we don't see at the last point
                        if (paths[k][i+1] == paths[j][i]) & (paths[k][i] == paths[j][i+1]):  # Check
                            print('X: ', paths[k][i][1], paths[k][i+1][1])
                            if paths[k][i][1] == paths[k][i+1][1]:  # If the moving along Y axes
                                paths[k].insert(i+1, tuple(map(operator.add, paths[k][i], (0, 1))))
                                paths[k].insert(i+2, tuple(map(operator.add, paths[k][i+1], (0, -1))))
                                print('Along x: ', paths[k][i+1])
                            else:
                                paths[k].insert(i+1, tuple(map(operator.add, paths[k][i], (1, 0))))
                                paths[k].insert(i+2, tuple(map(operator.add, paths[k][i+1], (-1, 0))))
                                print('Along y: ', paths[k][i + 1])
                            # We don't need check for i not eq 0 bcs we assume that all
                            # agents was placed in different coordinates as first point

                            print('Paths after: ')
                            print(paths[0])
                            print('\n')
                            print(paths[1])

                            # TODO: recalculate max_len
                            print('Swap between agents')
                            i -= 1
                            continue
                j += 1
            k += 1
        i += 1

    print('Paths after: ')
    print(paths[0])
    print(paths[1])


def start():
    a = GridWorld(20, 30)
    # a = GridWorld(5, 10)

    vis = Visualize(a)

    # Collision test
    a.add_agents([(2, 22, 12, 2), (16, 20, 4, 0), (6, 19, 5, 3), (4, 25, 11, 25), (11, 25, 4, 25), (9, 22, 13, 1)])

    a.add_rocks([(2, 0), (2, 1), (2, 2), (2, 3), (2, 4), (2, 5), (2, 6), (2, 7), (2, 8), (2, 9)])
    a.add_rocks([(3, 0), (3, 1), (3, 2), (3, 3), (3, 4), (3, 5), (3, 6), (3, 7), (3, 8), (3, 9)])

    a.add_rocks([(6, 0), (6, 1), (6, 2), (6, 3), (6, 4), (6, 5), (6, 6), (6, 7), (6, 8), (6, 9)])
    a.add_rocks([(7, 0), (7, 1), (7, 2), (7, 3), (7, 4), (7, 5), (7, 6), (7, 7), (7, 8), (7, 9)])

    a.add_rocks([(10, 0), (10, 1), (10, 2), (10, 3), (10, 4), (10, 5), (10, 6), (10, 7), (10, 8), (10, 9)])
    a.add_rocks([(11, 0), (11, 1), (11, 2), (11, 3), (11, 4), (11, 5), (11, 6), (11, 7), (11, 8), (11, 9)])

    a.add_rocks([(14, 0), (14, 1), (14, 2), (14, 3), (14, 4), (14, 5), (14, 6), (14, 7), (14, 8), (14, 9)])
    a.add_rocks([(15, 0), (15, 1), (15, 2), (15, 3), (15, 4), (15, 5), (15, 6), (15, 7), (15, 8), (15, 9)])

    vis.draw_world()
    vis.draw_agents()

    vis.canvas.pack()
    vis.canvas.update()
    vis.canvas.after(1000)

    actions = []
    paths = []

    for i in range(1, len(a.aindx_cpos.keys())+1):
        paths.append(astar.find_path(a.get_nbor_cells,
                                     a.aindx_cpos[i],
                                     a.aindx_goal[i],
                                     lambda cell: 1,
                                     lambda cell: not a.is_blocked(cell[0], cell[1]))[0][1:])

    check_crossings(paths)

    for num, path in enumerate(paths, 1):
        actions.append(list(reversed(a.path_to_action(num, path))))
        vis.draw_path(path, num)

    print('Actions: ', actions)

    finished = False
    while not finished:
        agent_number = 1
        finished = True
        for action in actions:
            if action:
                finished = False
                a.parallel_step(agent_number, action.pop())
            agent_number += 1
        a.check_collision()

        vis.canvas.update()
        vis.canvas.after(100)

    vis.canvas.after(3000)


if __name__ == '__main__':
    start()
