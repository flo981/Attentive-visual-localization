import orbslam_subsciber as orbsub
import vocus2_subscriber as vsub


if __name__ == '__main__':
    bins()

def bins():
    orbbins = orbsub.Subscriber()
    vocusbins = vsub.extractPOI()
    #print "orb: ", orbbins , " // vocus: ",   vocusbins
    if orbbins == vocusbins:
        #print "true"
        return vocusbins
    else:
        return False, False
