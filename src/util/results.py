import datetime

from context_manager.db_context_manager import DBContextManager
from util.constants import QUERIES


def export_results(data, controller_name, trajectory_name, database_path):
    def get_table_name(controller, trajectory, date_time):
        return '_'.join([controller,
                         trajectory,
                         date_time.strftime('%Y_%m_%d_%H_%M_%S')])

    date_time = datetime.datetime.now()

    with DBContextManager(database_path) as cursor:
        table_name = get_table_name(
            controller_name,
            trajectory_name,
            date_time
        )

        cursor.execute(QUERIES['create_sims'])
        cursor.execute(QUERIES['insert_sim'], (table_name, date_time))
        cursor.execute(QUERIES['create_sim'].format(table_name))

        for i in range(len(data['t'])):
            cursor.execute(
                QUERIES['insert_data'].format(table_name),
                (data['t'][i], data['x'][i], data['x_ref'][i],
                 data['y'][i], data['y_ref'][i], data['theta'][i],
                 data['theta_ref'][i], data['v_c'][i], data['w_c'][i])
            )
