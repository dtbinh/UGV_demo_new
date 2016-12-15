import gtk
import pygtk

class Demo2Window(gtk.Window):

    '''
    Test module to select place for semi connected vehicle to go
    '''

    def __init__(self):
        gtk.Window.__init__(self)
        self.set_size_request(1820, 950)

        self.l_button = gtk.Button('left')
        self.r_button = gtk.Button('right')
        self.go_button = gtk.Button('Go!')

        

win = Demo2Window()
win.show_all()
gtk.main()