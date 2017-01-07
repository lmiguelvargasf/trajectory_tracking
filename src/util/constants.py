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
}
