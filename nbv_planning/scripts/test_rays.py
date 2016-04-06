import numpy as np
import matplotlib.pyplot as plt
import math

free = 0.0
occupied = 1-free
unob = 0.5

odd=np.array([0.35]*25)
#odd[10]=occupied-0.0005
#odd[8]=0.6
#odd[9]=0.7
odd[10]=0.794999#
odd[11:]=0.5
odd[16:18]=0.05
#odd[21:]=0.5

odd=np.array([0.5]*25)
#odd[13]=.85
#odd[14:]=0.4234

p=[np.array([1,0])]
fucker=[1]
# Do one round on the po for the initialised
p_o=[]
#p_o = [np.array([free,occupied])]
p_o = [np.array([odd[0],1-odd[0]])]

next2=np.array([odd[0]  * p_o[-1][0],
                (1-odd[0]) * p_o[-1][1]])
next2/=next2[0]+next2[1]
p_o = [next2]
p_o = [np.array([free,occupied])]

obs=1.0
for i in range(1,25):
    t = 0.999**len(p)
    prev =  p[-1]
    trans = np.array([[t,1-t],[0.1,0.9]])
    obs=1-odd[i-1]
#    print "Updated transition matrix:",trans
    next=np.dot(prev,trans)
    next = np.array([(obs)*next[0],
                     (1-obs)*next[1]])#!
    next/=next[0]+next[1]

    #nasty:
    if next[0]>prev[0]:
        next=prev
    p.append(next)

    ########################################################
    t = 0.5 + (next[0]/2) # 8  fucker
    trans = np.array([[t, 1-t],[1-t, t]])
#    print "Updated transition p_o matrix\",trans
    #trans = np.eye(2)
    next2 = np.dot(p_o[-1],trans)
    fucker.append(next2[0])
    update=True
    if update:
        pz = odd[i]
        next2=np.array([pz * next2[0],
                        (1-pz) * next2[1]])
#        if odd[i]==0:
#            next2=np.array([0,1])
#        elif odd[i]==1:
#            next2=np.array([1,0])
#        elif next[0]>0.1:
#            if odd[i]>0.5:
#                #Prior: 0.75 	Post: 0.954545444436 	P(x): [ 0.0184759  0.9815241] 	Post^: 0.499999941749
#
#                next2=np.array([.7 * next2[0] / (1-odd[i]),
#                                .3 * next2[1] / (odd[i]) ] )
#                pass
#            elif odd[i]<0.5:
#                
#                next2=np.array([.3 * next2[0]/(1-odd[i]),
#                                .7 * next2[1]/(odd[i])])
#                pass
#        else:
#            next2=np.array([odd[i],1-odd[i]])
#    #    next2=np.array([next[0]  * next2[0],
#    #                    next[1] * next2[1]])
        if (next2[0]+next2[1])==0:
            # ah fuck it
            next2[0]=1
        else:
            next2/=next2[0]+next2[1]
    p_o.append(next2)

overall_gain=0
for prior,post,px,ohat in zip(odd, p_o,p,fucker):
    print "Prior:",prior,"\tPost:",post[0],"\tP(x):",px,"\tPost^:",ohat
#    print "Entropy before:"
    try:
        before = - prior*math.log(prior) - (1-prior)*math.log(1-prior)
    except:
        before = 0
#    print before
#    print "Entropy after: "
    try:
        after =  - post[0]*math.log(post[0]) - (post[1])*math.log(post[1])
    except:
        after=0
#    print after
#    print "Info gain:",
    gain=before-after
#    print gain
    overall_gain+=gain
print "="*10
print "View gain:", overall_gain
plt.plot(np.array(p)[:,0],'g--')
#plt.plot(fucker,'g--')
plt.plot(np.array(p_o)[:,0],'ro-',linewidth=2)
plt.plot(odd,'bs-')
plt.show()


