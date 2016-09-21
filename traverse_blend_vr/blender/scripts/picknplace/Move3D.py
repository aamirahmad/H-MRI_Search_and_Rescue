


import Rasterizer as R
from bge import logic, events
cont = logic.getCurrentController()
scn = logic.getCurrentScene()
own = cont.owner

R.showMouse(True)

scene = logic.getCurrentScene()
MouseOver = cont.sensors["MouseOver"]


keyboard = logic.keyboard	
JUST_ACTIVATED = logic.KX_INPUT_JUST_ACTIVATED  
JUST_RELEASED = logic.KX_INPUT_JUST_RELEASED

  
if __name__ == "__main__":
        if own['Draging'] and MouseOver.positive and own['Selected'] in scene.objects:
            if not own['Selected']['Aktief']:
                own['Selected']["Aktief"] = True
            # Vind de nieuwe positie van de muiscursor
            # in 3D coordinaten op het grondvlak
            HitObject = MouseOver.hitObject
            HitPosition = MouseOver.hitPosition
            #print(HitObject,HitPosition)
            Xhit = HitPosition[0]
            Yhit = HitPosition[1]
            Zhit = HitPosition[2]
            own['Xhit'] = Xhit
            own['Yhit'] = Yhit
            own['Zhit'] = Zhit
            	
            	
            # Zendt messageBody naar het huidige actieve object
            # met daarin de X,Y,Z- coordinaten
            # van het punt waar het voorwerp naar toe moet
            # gebruik ";" als scheidingteken
            Verplaats = cont.actuators["ZendMededeling"]
            Verplaats.subject = "Verplaatsen"
             
            Verplaats.propName = str(own['Selected'])
            #Verplaats.setBodyType(False)
            Inhoud = str(Xhit )+";"+ str(Yhit) +";" + str(Zhit)
            Verplaats.body = Inhoud 
            cont.activate(Verplaats)
