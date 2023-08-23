from robot_baxter import *
import rospy
def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("control_head")

    mylimb = MyLimb("left")
    rospy.on_shutdown(mylimb.clean_shutdown)
    mylimb.gripperclose()

if __name__ == '__main__':
    main()
