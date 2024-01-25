import numpy as np

class Fuzzy:
    def __init__(self):

        # self.frs_input = 0.3
        # self.brs_input = 0.8

        self.frs_near = [0.0, 0.25, 0.50]
        self.frs_middle = [0.25, 0.5, 0.75]
        self.frs_far = [0.5, 0.75, 1.0]
        self.brs_near = [0.0, 0.25, 0.50]
        self.brs_middle = [0.25, 0.5, 0.75]
        self.brs_far = [0.5, 0.75, 1.0]
        self.speed_slow = [0.1, 0.2, 0.3]
        self.speed_medium = [0.3, 0.4, 0.5]
        self.speed_fast = [0.5, 0.6, 0.7]
        self.go_left = [0.1, 0.3, 0.4]
        self.go_forward = [-0.1, 0.0, 1.0]
        self.go_right = [-0.1, -0.2, -0.3]
        self.speed = []
        self.direction =[]
        self.firing_strength =[]
        self.rules = {
            ("near", "near"): ("slow", "left"),
            ("near", "middle"): ("slow", "left"),
            ("near","far"): ("slow", "left"),
            ("middle", "near"): ("medium", "right"),
            ("middle", "middle"): ("medium", "forward"),
            ("middle", "far"): ("medium", "right"),
            ("far", "near"): ("fast", "right"),
            ("far", "middle"): ("fast", "right"),
            ("far", "far"): ("fast", "right")
        }
        self.raising_edge_values = []
        self.falling_edge_values = []
        self.frs_rules = []
        self.brs_rules = []
        self.rules_triggered =[]


    def rule_triggered(self):
        # if self.frs_input >= self.frs_near[0] and self.frs_input <= self.frs_near[2] and self.brs_input >= self.brs_near[0] and self.brs_input <= self.brs_near[2]:
        #     self.speed = np.mean(self.speed_slow)
        #     print('1=near, near')
        #     self.firing_strength = [np.min(self.frs_near), np.min(self.brs_near)]

        for i in range(len(self.rules_triggered)):
            # print(i)
            if self.rules_triggered[i] == 'nearnear':
                self.speed.append(np.mean(self.speed_slow))
                self.direction.append(np.mean(self.go_left))
                self.firing_strength.append(np.min((np.min(self.frs_near), np.min(self.brs_near))))

            if self.rules_triggered[i] == 'nearmiddle':
                self.speed.append(np.mean(self.speed_slow))
                self.direction.append(np.mean(self.go_left))
                self.firing_strength.append(np.min((np.min(self.frs_near), np.min(self.brs_middle))))

            if self.rules_triggered[i] == 'nearfar':
                self.speed.append(np.mean(self.speed_slow))
                self.direction.append(np.mean(self.go_left))
                self.firing_strength.append(np.min((np.min(self.frs_near), np.min(self.brs_far))))

            if self.rules_triggered[i] == 'middlenear':
                self.speed.append(np.mean(self.speed_medium))
                self.direction.append(np.mean(self.go_forward))
                self.firing_strength.append(np.min((np.min(self.frs_middle), np.min(self.brs_near))))

            if self.rules_triggered[i] == 'middlemiddle':
                self.speed.append(np.mean(self.speed_medium))
                self.direction.append(np.mean(self.go_forward))
                self.firing_strength.append(np.min((np.min(self.frs_middle), np.min(self.brs_middle))))

            if self.rules_triggered[i] == 'middlefar':
                self.speed.append(np.mean(self.speed_medium))
                self.direction.append(np.mean(self.go_forward))
                self.firing_strength.append(np.min((np.min(self.frs_middle), np.min(self.brs_far))))

            if self.rules_triggered[i] == 'farnear':
                self.speed.append(np.mean(self.speed_slow))
                self.direction.append(np.mean(self.go_right))
                self.firing_strength.append(np.min((np.min(self.frs_far), np.min(self.brs_near))))

            if self.rules_triggered[i] == 'farmiddle':
                self.speed.append(np.mean(self.speed_slow))
                self.direction.append(np.mean(self.go_right))
                self.firing_strength.append(np.min((np.min(self.frs_far), np.min(self.brs_middle))))

            if self.rules_triggered[i] == 'farfar':
                self.speed.append(np.mean(self.speed_slow))
                self.direction.append(np.mean(self.go_right))
                self.firing_strength.append(np.min((np.min(self.frs_far), np.min(self.brs_far))))
        temp1 = fuzzy.de_fuzzification()
        return temp1

    def de_fuzzification(self):
        # self.de_fuzzification_speed = (np.sum(np.multiply(self.speed, self.firing_strength)))/np.sum(self.firing_strength)
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

    def rising_edge_calculation(self,input,maximum,minimum):
        self.raising_edge_values.append((input - minimum)/(maximum - minimum))
    def falling_edge_calculation(self,input,maximum,minimum):
        self.falling_edge_values.append((maximum - input) / (maximum - minimum))

    def rising_edge_too_far(self, input):
        self.raising_edge_values.append(input)

    def falling_edge_too_far(self, input):
        self.falling_edge_values.append(input)

    def fuzzy_crisp(self, frs, brs):

        self.frs_input = frs
        self.brs_input = brs

        if self.frs_input < self.frs_near[0]:
            fuzzy.rising_edge_too_far(self.frs_near[0])
            self.frs_rules.append("near")

        if self.frs_input >= self.frs_near[0] and self.frs_input <= self.frs_near[1]:
            temp_list = [self.frs_near[0], self.frs_near[1]]
            maximum = np.max(temp_list)
            minimum = np.min(temp_list)
            fuzzy.rising_edge_calculation(self.frs_input, maximum, minimum)
            self.frs_rules.append("near")

        if self.frs_input > self.frs_near[1] and self.frs_input <= self.frs_near[2]:
            temp_list = [self.frs_near[1], self.frs_near[2]]
            maximum = np.max(temp_list)
            minimum = np.min(temp_list)
            fuzzy.falling_edge_calculation(self.frs_input, maximum, minimum)
            self.frs_rules.append("near")

            # maximum = np.max([self.frs_near[1], self.frs_near[2]])

        if self.frs_input >= self.frs_middle[0] and self.frs_input <= self.frs_middle[1]:
            temp_list = [self.frs_middle[0], self.frs_middle[1]]
            maximum = np.max(temp_list)
            minimum = np.min(temp_list)
            fuzzy.rising_edge_calculation(self.frs_input, maximum, minimum)
            self.frs_rules.append("middle")

        if self.frs_input > self.frs_middle[1] and self.frs_input <= self.frs_middle[2]:
            temp_list = [self.frs_middle[1], self.frs_middle[2]]
            maximum = np.max(temp_list)
            minimum = np.min(temp_list)
            fuzzy.falling_edge_calculation(self.frs_input, maximum, minimum)
            self.frs_rules.append("middle")

        if self.frs_input >= self.frs_far[0] and self.frs_input <= self.frs_far[1]:
            temp_list = [self.frs_far[0], self.frs_far[1]]
            maximum = np.max(temp_list)
            minimum = np.min(temp_list)
            fuzzy.rising_edge_calculation(self.frs_input, maximum, minimum)
            self.frs_rules.append("far")

        if self.frs_input > self.frs_far[1] and self.frs_input <= self.frs_far[2]:
            temp_list = [self.frs_far[1], self.frs_far[2]]
            maximum = np.max(temp_list)
            minimum = np.min(temp_list)
            fuzzy.falling_edge_calculation(self.frs_input, maximum, minimum)
            self.frs_rules.append("far")

        if self.frs_input > self.frs_far[2]:
            fuzzy.falling_edge_too_far(self.frs_far[2])
            self.frs_rules.append("far")

        if self.brs_input < self.brs_near[0]:
            fuzzy.falling_edge_too_far(self.brs_near[0])
            self.brs_rules.append("near")

        if self.brs_input >= self.brs_near[0] and self.brs_input <= self.brs_near[1]:
            temp_list = [self.brs_near[0], self.brs_near[1]]
            maximum = np.max(temp_list)
            minimum = np.min(temp_list)
            fuzzy.rising_edge_calculation(self.brs_input, maximum, minimum)
            self.brs_rules.append("near")

        if self.brs_input > self.brs_near[1] and self.brs_input <= self.brs_near[2]:
            temp_list = [self.brs_near[1], self.brs_near[2]]
            maximum = np.max(temp_list)
            minimum = np.min(temp_list)
            fuzzy.falling_edge_calculation(self.brs_input, maximum, minimum)
            self.brs_rules.append("near")

        if self.brs_input >= self.brs_middle[0] and self.brs_input <= self.brs_middle[1]:
            temp_list = [self.brs_middle[0], self.brs_middle[1]]
            maximum = np.max(temp_list)
            minimum = np.min(temp_list)
            fuzzy.rising_edge_calculation(self.brs_input, maximum, minimum)
            self.brs_rules.append("middle")

        if self.brs_input > self.brs_middle[1] and self.brs_input <= self.brs_middle[2]:
            temp_list = [self.brs_middle[1], self.brs_middle[2]]
            maximum = np.max(temp_list)
            minimum = np.min(temp_list)
            fuzzy.falling_edge_calculation(self.brs_input, maximum, minimum)
            self.brs_rules.append("middle")

        if self.brs_input >= self.brs_far[0] and self.brs_input <= self.brs_far[1]:
            temp_list = [self.brs_far[0], self.brs_far[1]]
            maximum = np.max(temp_list)
            minimum = np.min(temp_list)
            fuzzy.rising_edge_calculation(self.brs_input, maximum, minimum)
            self.brs_rules.append("far")

        if self.brs_input > self.brs_far[1] and self.brs_input <= self.brs_far[2]:
            temp_list = [self.brs_far[1], self.brs_far[2]]
            maximum = np.max(temp_list)
            minimum = np.min(temp_list)
            fuzzy.falling_edge_calculation(self.brs_input, maximum, minimum)
            self.brs_rules.append("far")

        # if self.brs_input > self.brs_far[2]:
        #     temp_list = [self.brs_far[1], self.brs_far[2]]
        #     maximum = np.max(temp_list)
        #     minimum = np.min(temp_list)
        #     fuzzy.falling_edge_calculation(self.brs_input, maximum, minimum)
        #     self.brs_rules.append("far")

        if self.brs_input > self.brs_far[2]:
            fuzzy.falling_edge_too_far(self.brs_far[2])
            self.brs_rules.append("far")

        temp3 = fuzzy.rules_figured()
        return temp3

    def rules_figured(self):
        for i in range(len(self.frs_rules)):
            for j in range(len(self.brs_rules)):
               self.rules_triggered.append(self.frs_rules[i] + self.brs_rules[j])
        temp2 = fuzzy.rule_triggered()
        return temp2
fuzzy = Fuzzy()
# fuzzy.fuzzy_crisp()


