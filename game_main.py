import sys
import rospy


from car import Car
from game_manager import Game


def main(args):
    rospy.init_node('game_node', anonymous=True)
    car = Car()
    game = Game(car)
    backsoundbool = True

    while True:
        if backsoundbool:
            backsoundbool = False
            #background_sound = pygame.mixer.Sound("Music/Hintergrundmusik.wav")
            #pygame.mixer.Channel(1).play(background_sound, loops = -1)


        game.loop()

if __name__ == '__main__':
    main(sys.argv)

