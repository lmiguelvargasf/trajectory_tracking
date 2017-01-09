#!/usr/bin/env python
QUERIES = {
    'select_sim':
        """
        SELECT name FROM simulations
        ORDER BY creation_datetime DESC
        """,
    'select_data':
        """
        SELECT * FROM {}
        """,
}

TITLES = {
    'x_vs_t': r'$x\ {\rm and}\ x_{ref}\ {\rm vs}\ t$',
    'x_error': r'$x_{error}\ {\rm vs}\ t$',
    'y_vs_t': r'$y\ {\rm and}\ y_{ref}\ {\rm vs}\ t$',
    'y_error': r'$y_{error}\ {\rm vs}\ t$',
    'theta_vs_t': r'$\theta,\ \theta_{ref}\ {\rm vs}\ t$',
    'theta_error': r'$\theta_{error}\ {\rm vs}\ t$',
    'trajectory': r'${\rm followed\ trajectory\ vs\ reference\ trajectory}$',
    'trajectories': r'${\rm followed\ trajectories\ vs\ reference\ trajectory}$',
    'v_vs_t': r'$v_{c}\ {\rm vs}\ t$',
    'w_vs_t': r'$\omega_{c}\ {\rm vs}\ t$',
    'x_n_y': r'${\rm results - }\ x\ {\rm and}\ y$',
    'theta_n_trajectory': r'${\rm results - }\ \theta\ {\rm and\ trajectory}$',
    'v_n_w': r'${\rm results - }\ v_{c}\ {\rm and}\ \omega_{c}$'
}

LABELS = {
    't': r'$t[{\rm s}]$',
    'x': r'$x[{\rm m}]$',
    'x_error': r'$x_{error}[{\rm m}]$',
    'y': r'$y[{\rm m}]$',
    'y_error': r'$y_{error}[{\rm m}]$',
    'theta': r'$\theta[{\rm rad}]$',
    'theta_error': r'$\theta_{error}[{\rm rad}]$',
    'v': r'$v_{c}[{\rm m/s}]$',
    'w': r'$\omega_{c}[{\rm rad/s}]$',
}

PLOT = {
    'line_width': 2,
    'fig_title_size': 21,
    'plot_title_size': 19,
    'axis_label_size': 17,
}

COLORS = {
    'ref': '--r',
    'line_0': 'b',
    'line_1': 'g',
    'line_2': 'c',
    'line_3': 'k',
    'line_4': 'm',
    'line_5': 'y',
}

ARRAY_NAMES = ('t', 'x', 'x_ref', 'y', 'y_ref', 'theta', 'theta_ref', 'v_c', 'w_c', 'zeros')
