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
          w_c REAL NOT NULL
        )
        """,
    'insert_data':
        """
        INSERT INTO {}
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
    'create_sims':
        """
        CREATE TABLE IF NOT EXISTS simulations
        (
          id INTEGER PRIMARY KEY,
          name VARCHAR(255) NOT NULL UNIQUE,
          creation_datetime DATETIME NOT NULL
        )
        """,
    'insert_sim':
        """
        INSERT INTO simulations
        (name, creation_datetime)
        VALUES (?, ?)
        """,
}
