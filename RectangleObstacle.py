class RectangleObstacle:
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def check_agent_collision(self, agent_x, agent_y, agent_radius):
        rect_left = self.x - self.width / 2
        rect_right = self.x + self.width / 2
        rect_bottom = self.y - self.height / 2
        rect_top = self.y + self.height / 2

        # 원의 중심이 사각형 내부에 있는 경우, 충돌한다
        if rect_left <= agent_x <= rect_right and rect_bottom <= agent_y <= rect_top:
            return True

        # 원의 중심이 사각형 바깥에 있을 때, 사각형의 가장 가까운 경계와 원의 중심 사이의 거리를 계산한다
        closest_x = rect_left if agent_x < rect_left else rect_right if agent_x > rect_right else agent_x
        closest_y = rect_bottom if agent_y < rect_bottom else rect_top if agent_y > rect_top else agent_y

        dist_x = agent_x - closest_x
        dist_y = agent_y - closest_y

        # 거리와 원의 반지름을 비교한다
        return (dist_x * dist_x + dist_y * dist_y) <= (agent_radius * agent_radius)