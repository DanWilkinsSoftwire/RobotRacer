#!/usr/bin/env python3
"""Camera web server + video recording.

Follows the SunFounder PiCar-X record tutorial:
https://docs.sunfounder.com/projects/picar-x-v20/en/latest/python/python_record.html

Vilib.display(web=True) starts an MJPEG server inside Vilib (no Flask app of our
own). Once running, open the live stream from any device on the same network:

    http://<robot-ip>:9000/mjpg        e.g. http://picar.local:9000/mjpg

Keyboard controls (in the terminal running this script):
    Q: record / pause / continue
    E: stop (saves the .avi)
    Ctrl + C: quit
"""

from time import sleep, strftime, localtime
from vilib import Vilib
import readchar
import os

manual = '''
Live stream:  http://<robot-ip>:9000/mjpg

Press keys to control recording:
    Q: record/pause/continue
    E: stop
    Ctrl + C: quit
'''


def print_overwrite(msg, end='', flush=True):
    print('\r\033[2K', end='', flush=True)
    print(msg, end=end, flush=True)


def main():
    rec_flag = 'stop'  # start, pause, stop
    vname = None
    username = os.getlogin()

    Vilib.rec_video_set["path"] = f"/home/{username}/Videos/"  # save path

    Vilib.camera_start(vflip=False, hflip=False)
    Vilib.display(local=False, web=True)  # web=True serves the MJPEG stream
    sleep(0.8)  # wait for startup

    print(manual)
    while True:
        key = readchar.readkey().lower()
        # record / pause / continue
        if key == 'q':
            if rec_flag == 'stop':
                rec_flag = 'start'
                vname = strftime("%Y-%m-%d-%H.%M.%S", localtime())
                Vilib.rec_video_set["name"] = vname
                Vilib.rec_video_run()
                Vilib.rec_video_start()
                print_overwrite('rec start ...')
            elif rec_flag == 'start':
                rec_flag = 'pause'
                Vilib.rec_video_pause()
                print_overwrite('pause')
            elif rec_flag == 'pause':
                rec_flag = 'start'
                Vilib.rec_video_start()
                print_overwrite('continue')
        # stop
        elif key == 'e' and rec_flag != 'stop':
            rec_flag = 'stop'
            Vilib.rec_video_stop()
            print_overwrite("The video saved as %s%s.avi" % (
                Vilib.rec_video_set["path"], vname), end='\n')
        # quit
        elif key == readchar.key.CTRL_C:
            Vilib.camera_close()
            print('\nquit')
            break

        sleep(0.1)


if __name__ == "__main__":
    main()
