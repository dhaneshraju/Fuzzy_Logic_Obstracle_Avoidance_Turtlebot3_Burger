import numpy as np
import pandas as pd


class Obstacle:
    def __init__(self):
        self.fl_near = [0.0, 0.25, 0.50]
        self.fl_middle = [0.25, 0.5, 0.75]
        self.fl_far = [0.5, 0.75, 1.0]
        self.f_near = [0.0, 0.25, 0.50]
        self.f_middle = [0.25, 0.5, 0.75]
        self.f_far = [0.5, 0.75, 1.0]
        self.fr_near = [0.0, 0.25, 0.50]
        self.fr_middle = [0.25, 0.5, 0.75]
        self.fr_far = [0.5, 0.75, 1.0]
        self.speed_slow = [0.1, 0.2, 0.3]
        self.speed_medium = [0.3, 0.4, 0.5]
        self.speed_fast = [0.5, 0.6, 0.7]
        self.go_left = [0.1, 0.3, 0.4]
        self.go_forward = [-0.1, 0.0, 1.0]
        self.go_right = [-0.1, -0.2, -0.3]
        self.rules={
            ("near", "near", "near"): ("slow", "left"),
            ("near", "near", "middle"): ("slow", "left"),
            ("near", "near", "far"): ("medium", "forward"),
            ("near", "middle", "near"): ("slow", "left"),
            ("near", "middle", "middle"): ("slow", "left"),
            ("near", "middle", "far"): ("slow", "left"),
            ("near", "far", "near"): ("medium", "left"),
            ("near", "far", "middle"): ("medium", "left"),
            ("near", "far", "far"): ("medium", "left"),
            ("middle", "near", "near"): ("slow", "left"),
            ("middle", "near", "middle"): ("slow", "left"),
            ("middle", "near", "far"): ("slow", "right"),
            ("middle", "middle", "near"): ("slow", "left"),
            ("middle", "middle", "middle"): ("slow", "left"),
            ("middle", "middle", "far"): ("medium", "forward"),
            ("middle", "far", "near"): ("slow", "left"),
            ("middle", "far", "middle"): ("slow", "left"),
            ("middle", "far", "far"): ("medium", "left"),
            ("far", "near", "near"): ("slow", "right"),
            ("far", "near", "middle"): ("slow", "right"),
            ("far", "near", "far"): ("medium", "right"),
            ("far", "middle", "near"): ("medium", "right"),
            ("far", "middle", "middle"): ("medium", "right"),
            ("far", "middle", "far"): ("medium", "right"),
            ("far", "far", "near"): ("medium", "left"),
            ("far", "far", "middle"): ("medium", "left"),
            ("far", "far", "far"): ("fast", "front"),
        }
        self.fl_rules = []
        self.f_rules = []
        self.fr_rules = []
        self.raising_edge_values = []
        self.falling_edge_values = []
        self.rules_triggered = []
        self.speed = []
        self.direction = []
        self.firing_strength = []

    def rule_triggered(self):
        for i in range(len(self.rules_triggered)):
            if self.rules_triggered[i] == "nearnearnear":
                self.speed.append(np.mean(self.speed_slow))
                self.direction.append(np.mean(self.go_left))
                self.firing_strength.append(np.min((np.min(self.fl_near), np.min(self.f_near), np.min(self.fr_near))))

            if self.rules_triggered[i] == "nearnearmiddle":
                self.speed.append(np.mean(self.speed_medium))
                self.direction.append(np.mean(self.go_left))
                self.firing_strength.append(np.min((np.min(self.fl_near), np.min(self.f_near), np.min(self.fr_middle))))

            if self.rules_triggered[i] == "nearnearfar":
                self.speed.append(np.mean(self.speed_slow))
                self.direction.append(np.mean(self.go_forward))
                self.firing_strength.append(np.min((np.min(self.fl_near), np.min(self.f_near), np.min(self.fr_far))))

            if self.rules_triggered[i] == "nearmiddlenear":
                self.speed.append(np.mean(self.speed_slow))
                self.direction.append(np.mean(self.go_left))
                self.firing_strength.append(np.min((np.min(self.fl_near), np.min(self.f_middle), np.min(self.fr_near))))

            if self.rules_triggered[i] == "nearmiddlemiddle":
                self.speed.append(np.mean(self.speed_slow))
                self.direction.append(np.mean(self.go_left))
                self.firing_strength.append(np.min((np.min(self.fl_near), np.min(self.f_middle), np.min(self.fr_middle))))

            if self.rules_triggered[i] == "nearmiddlefar":
                self.speed.append(np.mean(self.speed_slow))
                self.direction.append(np.mean(self.go_left))
                self.firing_strength.append(np.min((np.min(self.fl_near), np.min(self.f_middle), np.min(self.fr_far))))

            if self.rules_triggered[i] == "nearfarnear":
                self.speed.append(np.mean(self.speed_medium))
                self.direction.append(np.mean(self.go_left))
                self.firing_strength.append(np.min((np.min(self.fl_near), np.min(self.f_far), np.min(self.fr_near))))

            if self.rules_triggered[i] == "nearfarmiddle":
                self.speed.append(np.mean(self.speed_medium))
                self.direction.append(np.mean(self.go_left))
                self.firing_strength.append(np.min((np.min(self.fl_near), np.min(self.f_far), np.min(self.fr_middle))))

            if self.rules_triggered[i] == "nearfarfar":
                self.speed.append(np.mean(self.speed_medium))
                self.direction.append(np.mean(self.go_left))
                self.firing_strength.append(np.min((np.min(self.fl_near), np.min(self.f_far), np.min(self.fr_far))))

            if self.rules_triggered[i] == "middlenearnear":
                self.speed.append(np.mean(self.speed_slow))
                self.direction.append(np.mean(self.go_left))
                self.firing_strength.append(np.min((np.min(self.fl_middle), np.min(self.f_near), np.min(self.fr_near))))

            if self.rules_triggered[i] == "middlenearmiddle":
                self.speed.append(np.mean(self.speed_slow))
                self.direction.append(np.mean(self.go_left))
                self.firing_strength.append(np.min((np.min(self.fl_middle), np.min(self.f_near), np.min(self.fr_middle))))

            if self.rules_triggered[i] == "middlenearfar":
                self.speed.append(np.mean(self.speed_slow))
                self.direction.append(np.mean(self.go_right))
                self.firing_strength.append(np.min((np.min(self.fl_middle), np.min(self.f_near), np.min(self.fr_far))))

            if self.rules_triggered[i] == "middlemiddlenear":
                self.speed.append(np.mean(self.speed_slow))
                self.direction.append(np.mean(self.go_left))
                self.firing_strength.append(np.min((np.min(self.fl_middle), np.min(self.f_middle), np.min(self.fr_near))))

            if self.rules_triggered[i] == "middlemiddlemiddle":
                self.speed.append(np.mean(self.speed_slow))
                self.direction.append(np.mean(self.go_left))
                self.firing_strength.append(np.min((np.min(self.fl_middle), np.min(self.f_middle), np.min(self.fr_middle))))

            if self.rules_triggered[i] == "middlemiddlefar":
                self.speed.append(np.mean(self.speed_medium))
                self.direction.append(np.mean(self.go_forward))
                self.firing_strength.append(np.min((np.min(self.fl_middle), np.min(self.f_middle), np.min(self.fr_far))))

            if self.rules_triggered[i] == "middlefarnear":
                self.speed.append(np.mean(self.speed_slow))
                self.direction.append(np.mean(self.go_left))
                self.firing_strength.append(np.min((np.min(self.fl_middle), np.min(self.f_far), np.min(self.fr_near))))

            if self.rules_triggered[i] == "middlefarmiddle":
                self.speed.append(np.mean(self.speed_slow))
                self.direction.append(np.mean(self.go_left))
                self.firing_strength.append(np.min((np.min(self.fl_middle), np.min(self.f_far), np.min(self.fr_middle))))

            if self.rules_triggered[i] == "middlefarfar":
                self.speed.append(np.mean(self.speed_medium))
                self.direction.append(np.mean(self.go_left))
                self.firing_strength.append(np.min((np.min(self.fl_middle), np.min(self.f_far), np.min(self.fr_far))))

            if self.rules_triggered[i] == "farnearnear":
                self.speed.append(np.mean(self.speed_slow))
                self.direction.append(np.mean(self.go_right))
                self.firing_strength.append(np.min((np.min(self.fl_far), np.min(self.f_near), np.min(self.fr_near))))

            if self.rules_triggered[i] == "farnearmiddle":
                self.speed.append(np.mean(self.speed_slow))
                self.direction.append(np.mean(self.go_right))
                self.firing_strength.append(np.min((np.min(self.fl_far), np.min(self.f_near), np.min(self.fr_middle))))

            if self.rules_triggered[i] == "farnearfar":
                self.speed.append(np.mean(self.speed_medium))
                self.direction.append(np.mean(self.go_right))
                self.firing_strength.append(np.min((np.min(self.fl_far), np.min(self.f_near), np.min(self.fr_far))))

            if self.rules_triggered[i] == "farmiddlenear":
                self.speed.append(np.mean(self.speed_medium))
                self.direction.append(np.mean(self.go_right))
                self.firing_strength.append(np.min((np.min(self.fl_far), np.min(self.f_middle), np.min(self.fr_near))))

            if self.rules_triggered[i] == "farmiddlemiddle":
                self.speed.append(np.mean(self.speed_medium))
                self.direction.append(np.mean(self.go_right))
                self.firing_strength.append(np.min((np.min(self.fl_far), np.min(self.f_middle), np.min(self.fr_middle))))

            if self.rules_triggered[i] == "farmiddlefar":
                self.speed.append(np.mean(self.speed_medium))
                self.direction.append(np.mean(self.go_right))
                self.firing_strength.append(np.min((np.min(self.fl_far), np.min(self.f_middle), np.min(self.fr_far))))

            if self.rules_triggered[i] == "farfarnear":
                self.speed.append(np.mean(self.speed_medium))
                self.direction.append(np.mean(self.go_left))
                self.firing_strength.append(np.min((np.min(self.fl_far), np.min(self.f_far), np.min(self.fr_near))))

            if self.rules_triggered[i] == "farfarmiddle":
                self.speed.append(np.mean(self.speed_medium))
                self.direction.append(np.mean(self.go_left))
                self.firing_strength.append(np.min((np.min(self.fl_far), np.min(self.f_far), np.min(self.fr_middle))))

            if self.rules_triggered[i] == "farfarfar":
                self.speed.append(np.mean(self.speed_fast))
                self.direction.append(np.mean(self.go_forward))
                self.firing_strength.append(np.min((np.min(self.fl_far), np.min(self.f_far), np.min(self.fr_far))))

        # print(self.speed)
        # print(self.direction)
        temp1 = oa.obstracle_defuzzification()
        return temp1

    def obstracle_defuzzification(self):
        a = [ x * y for x, y in zip(self.speed, self.firing_strength)]
        b = sum(a)
        c = sum(self.firing_strength)
        self.de_fuzzification_speed = b/c
        # print(f"Linear:{self.de_fuzzification_speed}")

        # self.de_fuzzification_direction = (np.sum(np.multiply(self.direction, self.firing_strength)))/np.sum(self.firing_strength)
        a = [x * y for x, y in zip(self.direction, self.firing_strength)]
        b = sum(a)
        c = sum(self.firing_strength)
        self.de_fuzzification_direction = b/c
        # print(f"Angular:{self.de_fuzzification_direction}")

        return self.de_fuzzification_speed, self.de_fuzzification_direction




    def rising_edge_calculation(self, input, maximum, minimum):

        self.raising_edge_values.append((input - minimum)/(maximum - minimum))

    def falling_edge_calculation(self, input, maximum, minimum):

        self.falling_edge_values.append((maximum - input) / (maximum - minimum))

    def rising_edge_too_far(self, input):

        self.raising_edge_values.append(input)

    def falling_edge_too_far(self, input):

        self.falling_edge_values.append(input)


    def oa_calculation(self, fleft, fright, front1, front2,fro):
        x = [front1, front2]
        x_x = x[0] + x[1]
        x = x_x/len(x)
        self.fl = fleft
        self.f = x
        self.fr = fright
        # print(self.fl)
        # print(self.f)
        # print(self.fr)
        if self.fl < self.fl_near[1]:
            self.fl_rules.append("near")

        if self.fl > self.fl_near[1] and self.fl <= self.fl_near[2]:
            temp_list = [self.fl_near[1], self.fl_near[2]]
            maximum = np.max(temp_list)
            minimum = np.min(temp_list)
            oa.falling_edge_calculation(self.fl, maximum, minimum)
            self.fl_rules.append("near")

        if self.fl >= self.fl_middle[0] and self.fl <= self.fl_middle[1]:
            temp_list = [self.fl_middle[0], self.fl_middle[1]]
            maximum = np.max(temp_list)
            minimum = np.min(temp_list)
            oa.rising_edge_calculation(self.fl, maximum, minimum)
            self.fl_rules.append("middle")

        if self.fl > self.fl_middle[1] and self.fl <= self.fl_middle[2]:
            temp_list = [self.fl_middle[1], self.fl_middle[2]]
            maximum = np.max(temp_list)
            minimum = np.min(temp_list)
            oa.falling_edge_calculation(self.fl, maximum, minimum)
            self.fl_rules.append("middle")

        if self.fl >= self.fl_far[0] and self.fl <= self.fl_far[1]:
            temp_list = [self.fl_far[0], self.fl_far[1]]
            maximum = np.max(temp_list)
            minimum = np.min(temp_list)
            oa.rising_edge_calculation(self.fl, maximum, minimum)
            self.fl_rules.append("far")

        if self.fl > self.fl_far[1]:
            self.fl_rules.append("far")


        if self.f < self.f_near[1]:
            self.f_rules.append("near")


        if self.f > self.f_near[1] and self.f <= self.f_near[2]:
            temp_list = [self.f_near[1], self.f_near[2]]
            maximum = np.max(temp_list)
            minimum = np.min(temp_list)
            oa.falling_edge_calculation(self.f, maximum, minimum)
            self.f_rules.append("near")

        if self.f >= self.f_middle[0] and self.f <= self.f_middle[1]:
            temp_list = [self.f_middle[0], self.f_middle[1]]
            maximum = np.max(temp_list)
            minimum = np.min(temp_list)
            oa.rising_edge_calculation(self.f, maximum, minimum)
            self.f_rules.append("middle")

        if self.f > self.f_middle[1] and self.f <= self.f_middle[2]:
            temp_list = [self.f_middle[1], self.f_middle[2]]
            maximum = np.max(temp_list)
            minimum = np.min(temp_list)
            oa.falling_edge_calculation(self.f, maximum, minimum)
            self.f_rules.append("middle")

        if self.f >= self.f_far[0] and self.f <= self.f_far[1]:
            temp_list = [self.f_far[0], self.f_far[1]]
            maximum = np.max(temp_list).

            minimum = np.min(temp_list)
            oa.rising_edge_calculation(self.f, maximum, minimum)
            self.f_rules.append("far")


        if self.f > self.f_far[1]:
            self.f_rules.append("far")

        if self.fr < self.fr_near[1]:
            self.fr_rules.append("near")

        mapped_number1 = 0.1 + (front1 / 100000) * (2.0 - 0.1)
        mapped_number2 = 0.1 + (fro / 100000) * (2.0 - 0.1)
        temp3 = [mapped_number1, mapped_number2]


        if self.fr > self.fr_near[1] and self.fr <= self.fr_near[2]:
            temp_list = [self.fr_near[1], self.fr_near[2]]
            maximum = np.max(temp_list)
            minimum = np.min(temp_list)
            oa.falling_edge_calculation(self.fr, maximum, minimum)
            self.fr_rules.append("near")

        if self.fr >= self.fr_middle[0] and self.fr <= self.fr_middle[1]:
            temp_list = [self.fr_middle[0], self.fr_middle[1]]
            maximum = np.max(temp_list)
            minimum = np.min(temp_list)
            oa.rising_edge_calculation(self.fr, maximum, minimum)
            self.fr_rules.append("middle")

        if self.fr > self.fr_middle[1] and self.fr <= self.fr_middle[2]:
            temp_list = [self.fr_middle[1], self.fr_middle[2]]
            maximum = np.max(temp_list)
            minimum = np.min(temp_list)
            oa.falling_edge_calculation(self.fr, maximum, minimum)
            self.fr_rules.append("middle")
        #
        if self.fr >= self.fr_far[0] and self.fr <= self.fr_far[1]:
            temp_list = [self.fr_far[0], self.fr_far[1]]
            maximum = np.max(temp_list)
            minimum = np.min(temp_list)
            oa.rising_edge_calculation(self.fr, maximum, minimum)
            self.fr_rules.append("far")


        if self.fr > self.fr_far[1]:
            self.fr_rules.append("far")

        # temp3 = oa.rules_figured()
        return temp3

    def rules_figured(self):
        for i in range (len(self.fl_rules)):
            for j in range(len(self.f_rules)):
                for k in range(len(self.fr_rules)):
                    self.rules_triggered.append(self.fl_rules[i] + self.f_rules[j] + self.fr_rules[k])
        temp2 = oa.rule_triggered()
        return temp2
oa = Obstacle()
