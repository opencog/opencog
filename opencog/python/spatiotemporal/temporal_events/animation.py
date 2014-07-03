from matplotlib.lines import Line2D
from matplotlib.ticker import AutoMinorLocator
from numpy.core.multiarray import zeros
from spatiotemporal.temporal_events.trapezium import TemporalEventTrapezium
from spatiotemporal.time_intervals import TimeInterval
from matplotlib import pyplot as plt
from matplotlib import animation

__author__ = 'keyvan'

x_axis = xrange(13)
zeros_13 = zeros(13)


class Animation(object):
    def __init__(self, event_a, event_b, event_c, plt=plt):
        self.event_a = event_a
        self.event_c = event_c
        self.event_b_length_beginning = event_b.beginning - event_b.a
        self.event_b_length_middle = self.event_b_length_beginning + event_b.ending - event_b.beginning
        self.event_b_length_total = event_b.b - event_b.ending
        self.plt = plt

        self.fig = plt.figure(1)
        self.ax_a_b = self.fig.add_subplot(4, 1, 1)
        self.ax_b_c = self.fig.add_subplot(4, 1, 2)
        self.ax_a_c = self.fig.add_subplot(4, 1, 3)
        self.ax_relations = self.fig.add_subplot(4, 1, 4)

        self.ax_a_b.set_xlim(0, 13)
        self.ax_a_b.set_ylim(0, 1)

        self.ax_b_c.set_xlim(0, 13)
        self.ax_b_c.set_ylim(0, 1)

        self.ax_a_c.set_xlim(0, 13)
        self.ax_a_c.set_ylim(0, 1)

        self.rects_a_b = self.ax_a_b.bar(x_axis, zeros_13)
        self.rects_b_c = self.ax_b_c.bar(x_axis, zeros_13)
        self.rects_a_c = self.ax_a_c.bar(x_axis, zeros_13)

        self.line_a = Line2D([], [])
        self.line_b = Line2D([], [])
        self.line_c = Line2D([], [])

        self.ax_relations.add_line(self.line_a)
        self.ax_relations.add_line(self.line_b)
        self.ax_relations.add_line(self.line_c)

        a = min(event_a.a, event_c.a) - self.event_b_length_total
        b = max(event_a.b, event_c.b)
        self.ax_relations.set_xlim(a, b + self.event_b_length_total)
        self.ax_relations.set_ylim(0, 1.1)

        # self.interval = TimeInterval(a, b, 150)

        self.interval = TimeInterval(a, b, 2)
        self.ax_a_b.xaxis.set_minor_formatter(self.ax_a_b.xaxis.get_major_formatter())
        self.ax_a_b.xaxis.set_minor_locator(AutoMinorLocator(2))
        self.ax_a_b.xaxis.set_ticklabels('poDedOP')
        self.ax_a_b.xaxis.set_ticklabels('mFsSfM', minor=True)

        self.ax_b_c.xaxis.set_minor_formatter(self.ax_b_c.xaxis.get_major_formatter())
        self.ax_b_c.xaxis.set_minor_locator(AutoMinorLocator(2))
        self.ax_b_c.xaxis.set_ticklabels('poDedOP')
        self.ax_b_c.xaxis.set_ticklabels('mFsSfM', minor=True)

        self.ax_a_c.xaxis.set_minor_formatter(self.ax_a_c.xaxis.get_major_formatter())
        self.ax_a_c.xaxis.set_minor_locator(AutoMinorLocator(2))
        self.ax_a_c.xaxis.set_ticklabels('poDedOP')
        self.ax_a_c.xaxis.set_ticklabels('mFsSfM', minor=True)

    def init(self):
        artists = []
        self.line_a.set_data(self.event_a, self.event_a.membership_function)
        self.line_b.set_data([], [])
        self.line_c.set_data(self.event_c, self.event_c.membership_function)
        artists.append(self.line_a)
        artists.append(self.line_b)
        artists.append(self.line_c)
        for rect, h in zip(self.rects_a_b, zeros_13):
            rect.set_height(h)
            artists.append(rect)
        for rect, h in zip(self.rects_b_c, zeros_13):
            rect.set_height(h)
            artists.append(rect)
        for rect, h in zip(self.rects_a_c, (self.event_a * self.event_c).to_list()):
            rect.set_height(h)
            artists.append(rect)
        return artists

    def animate(self, t):
        interval = self.interval
        B = TemporalEventTrapezium(interval[t], interval[t] + self.event_b_length_total,
                                   interval[t] + self.event_b_length_beginning,
                                   interval[t] + self.event_b_length_middle)
        plt.figure()
        B.plot().show()
        a_b = (self.event_a * B).to_list()
        b_c = (B * self.event_c).to_list()
        self.line_b.set_data(B, B.membership_function)
        artists = []
        for rect, h in zip(self.rects_a_b, a_b):
            rect.set_height(h)
            artists.append(rect)

        for rect, h in zip(self.rects_b_c, b_c):
            rect.set_height(h)
            artists.append(rect)

        artists.append(self.line_a)
        artists.append(self.line_b)
        artists.append(self.line_c)
        return artists

    def show(self):
        fr = len(self.interval) - 1
        anim = animation.FuncAnimation(self.fig, self.animate, init_func=self.init,
                                       frames=fr, interval=fr, blit=True)
        self.plt.show()

if __name__ == '__main__':
    anim = Animation(TemporalEventTrapezium(4, 8, 5, 7),
                     TemporalEventTrapezium(0, 10, 6, 9),
                     TemporalEventTrapezium(0.5, 11, 1, 3))
    #anim.show()
