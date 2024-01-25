global ei, ed, eprevious

class pid:
    def __init__(self):
        self.ei, self.ed, self.eprevious = 0, 0, 0

    def pidcontroller(self, cd):

        #pid error updating, dd is desired distance and cd is current distance
        #ei to be a global as 0,

        dd = 1.0
        error = dd - cd
        #eprev needs to be updated after
        # print("ei:", self.ei)
        # print("error:", error)
        self.ei = self.ei + error
        # print("ei:", self.ei)
        # print("eprevious:", self.eprevious)
        #ed also as a global set to 0
        self.ed = error - self.eprevious
        eprevious = error
        # print("ed:", self.ed)


        # print("eprevious:", self.eprevious)

        kp=0.1
        ki=0.00001
        kd=0.1

        actual_val = kp*error + ki*self.ei + kd*self.ed
        # print("actual_value", actual_val)

        return actual_val

pid_obj = pid()

