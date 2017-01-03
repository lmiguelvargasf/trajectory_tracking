#!/usr/bin/env python
QUERIES = {
    'create_sim':
        """
        CREATE TABLE IF NOT EXISTS {} (
        t REAL  NOT NULL PRIMARY KEY,
        x REAL NOT NULL,
        x_ref REAL NOT NULL,
        y REAL NOT NULL,
        y_ref REAL NOT NULL,
        theta REAL NOT NULL,
        theta_ref REAL NOT NULL,
        v_c REAL NOT NULL,
        w_c REAL NOT NULL)
        """,
    'insert_data':
        """
        INSERT INTO {} (t, x, x_ref, y, y_ref, theta, theta_ref, v_c, w_c)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
    'create_sims':
        """
        CREATE TABLE IF NOT EXISTS simulations (
        id INTEGER  NOT NULL PRIMARY KEY AUTOINCREMENT,
        name VARCHAR(255) NOT NULL UNIQUE,
        creation_datetime DATETIME)
        """,
    'insert_sim':
        """
        INSERT INTO simulations (name, creation_datetime)
        VALUES (?, ?)
        """,
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
