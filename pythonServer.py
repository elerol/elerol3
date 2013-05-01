import os
import BaseHTTPServer
import serial

serial_port_name = '/dev/ttyACM0'

class MyHandler(BaseHTTPServer.BaseHTTPRequestHandler):
        global serial_port
        def do_GET(self) :
                req_path,req_sep,req_get = self.path.partition("?")
                if req_path=="/" :
                        # The default file will be the usual index.html
                        #req_path = "index.html"
                        req_path = "app.html"
                if req_path.find('..')>=0 :
                        # Prevent a request outside the htdocs folder
                        self.send_error(404,'File Not Found: %s' % (req_path))
                        return
                if req_get!='' :
                        # Extract the parameters of the request.
                        # Convert the string into a dictionnary of requests.
                        req_get_dict = {}
                        for reqg in req_get.split('&') :
                                rk,rv = reqg.split('=')
                                req_get_dict[rk] = rv
###                     print "req=%s" % req_get_dict
                        # Check if the LEDs have to be changed...
                        if req_get_dict.has_key('led') and req_get_dict.has_key('led_value') :
                                # Send a message on the serial port
                                #cmd = "*l%01d%01d" % (int(req_get_dict['led']),int(req_get_dict['led_value']))
                                cmd= "%d" % (int(req_get_dict['led_value']))
###                             print "COMMANDE RECUE = : "
###                             print cmd
                                if serial_port!=None :
                                        #serial_port.write( cmd+"\n\r" )
                                        n=serial_port.write(chr(int(cmd)))
######                                  r=serial_port.read()
###                                     print "COMMANDE DE n char  SENT  SERIAL CHAR= : %s " %(n)
###                                     print r
                                        #print"COMMANDE DE RETOUR SERIAL RECUE INTEGER= : %d" % int(r)
                                #print cmd
                #full_filename = os.curdir + os.sep + "htdocs"+ os.sep + req_path
                full_filename = os.sep +"python" + os.sep + "htdocs" + os.sep + "testmoteur" +os.sep + req_path
                try:
                        ###print "\n\nPATH full filename= : %s" %(full_filename)
                        # Copy the file in htdocs to the network...
                        #echo 'TEST DE ECHO'
                        with open( full_filename, 'r' ) as f :
                                self.send_response(200)
                                self.end_headers()
                                self.wfile.write(f.read())
                        return
                except IOError:
                        # The requested file is not available...
                        print "File not found:req path %s" % (req_path)
                        print "\nPATH =: %s" % (full_filename)
                        self.send_error(404,'File Not Found: %s ' % (full_filename))

def main():
        global serial_port
        try:
                # Open the serial port
                serial_port = serial.Serial( port=serial_port_name, baudrate=9600, timeout=1, writeTimeout=0.5 )
                #serial_port = serial.Serial(0)  # on ouvre le 1er port serie disponible
                print serial_port.portstr       # on affiche son nom s'il en existe
        except serial.SerialException :
                print "Failed to open serial port %s! Using dummy mode" % serial_port_name
                serial_port = None
        # Start the web server
        try:
                server = BaseHTTPServer.HTTPServer(('', 8088), MyHandler)
                print 'started httpserver...'
                server.serve_forever()
        except KeyboardInterrupt:
                print '^C received, shutting down server'
                server.socket.close()
        if serial_port!=None:
                serial_port.close()

if __name__ == '__main__':
        main()

