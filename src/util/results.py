import datetime

from context_manager.db_context_manager import DBContextManager
from util.constants import QUERIES


def export_results(data, database_path):
    creation_datetime = datetime.datetime.now()

    with DBContextManager(database_path) as cursor:
        table_name = ('_'.join(['euler', 'linear', creation_datetime.strftime('%Y_%m_%d_%H_%M_%S')]))
        cursor.execute(QUERIES['create_sims'])

        cursor.execute(QUERIES['insert_sim'], (table_name, creation_datetime))

        cursor.execute(QUERIES['create_sim'].format(table_name))

        for i in range(len(data['t'])):
            cursor.execute(
                QUERIES['insert_data'].format(table_name),
                (data['t'][i], data['x'][i], data['x_ref'][i],
                 data['y'][i], data['y_ref'][i], data['theta'][i],
                 data['theta_ref'][i], data['v_c'][i], data['w_c'][i])
            )
