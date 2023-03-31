import json

from game_controller import GameController


"""
This is the main module where the game starts.
"""


def load_drones_settings():
    """
    Load drone settings from the json file.

    :return: List of dicts representing drone settings.
    """
    with open('drones.json', 'r') as f:
        drones = json.load(f)
    return drones
        

if __name__ == '__main__':
    drones = load_drones_settings()
    GameController(drones).main()
