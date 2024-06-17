# ETC LightHack Variation   

A working version of ETC lighthack that uses 6 encoders and a shift key. The lcd has been removed from this as is compatibility with any software other than EOS. This can easily be added back in, as most  of the code is still there.

# Setup

One of the encoders is set to work as an intensity wheel using the +% and -% keys. you will need to set these to 1% in the user settings of eos.

# Other encoder usage
I built this to work with fixtures that i use on a regular basis and therefore the parametres of control are bespoke to those which i find annoying to use a mouse for on Nomad.

Encoder 1 - Pan, Tilt
Encoder 2 - Zoom, Edge
Encoder 3 - Red, Green
Encoder 4 - Blue, Amber
Encoder 5 - White

The second parametre of each of the encoders is accessed by pressing the button on pin 8.

## Authors

- [@ETCLabs](https://github.com/ETCLabs/lighthack)


## FAQ

#### What board does it run on?

I currently have it running on a nano clone, but this doesnt work with the console due to the usb serial chip. I have had it running on an UNO as well.



