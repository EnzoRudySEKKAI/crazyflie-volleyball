import json

from game_controller import GameController



def load_drones_settings():
    with open('drones.json', 'r') as f:
        drones = json.load(f)
    return drones
        

if __name__ == '__main__':
    drones = load_drones_settings()
    GameController(drones).main()
