"""Disk-backed metrics. Ground truth is interpolated, never extrapolated.

Design reviewed against gpt_test's evaluator; this implementation streams records
through SQLite instead of retaining an entire experiment in Python memory.
"""
import argparse
import csv
import itertools
import json
import math
from pathlib import Path
import sqlite3


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def clean(value):
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [clean(v) for v in value]
    return None if isinstance(value, float) and not math.isfinite(value) else value


def measurement_window(value):
    """Validate a single clock epoch; bounds apply to source timestamps."""
    if value is None:
        return None
    if not isinstance(value, dict):
        raise ValueError('Measurement window must be an object')
    segment = value.get('segment')
    start, end = value.get('start_sim_s'), value.get('end_sim_s')
    if (not isinstance(segment, int) or isinstance(segment, bool) or segment < 0
            or not finite(start) or not finite(end) or not 0 <= start < end):
        raise ValueError('Window requires segment >= 0 and finite 0 <= start_sim_s < end_sim_s')
    return {'segment': segment, 'start_sim_s': float(start), 'end_sim_s': float(end),
            'duration_sim_s': float(end-start)}


def window_clause(window):
    if window is None:
        return '', []
    return ' AND segment=? AND t>=? AND t<=?', [
        window['segment'], window['start_sim_s'], window['end_sim_s']]


def in_window(segment, stamp, window):
    return window is None or (segment == window['segment']
                             and window['start_sim_s'] <= stamp <= window['end_sim_s'])


def position(row):
    p = tuple(row.get(k) for k in ('x', 'y', 'z'))
    return p if all(finite(v) for v in p) else None


def distance(a, b):
    return math.dist(a,b)


def cross_track(point, route):
    if not route:
        return None
    if len(route) == 1:
        return math.hypot(point[0]-route[0][0], point[1]-route[0][1])
    best = math.inf
    for a, b in zip(route, route[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = dx*dx+dy*dy
        fraction = max(0.0, min(1.0, ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/length)) if length else 0.0
        best = min(best, math.hypot(point[0]-a[0]-fraction*dx, point[1]-a[1]-fraction*dy))
    return best


def create_database(journal, database, max_bytes=None):
    connection = sqlite3.connect(database)
    connection.execute('PRAGMA cache_size=-8192')
    connection.execute('PRAGMA temp_store=FILE')
    connection.executescript('''
        DROP TABLE IF EXISTS obs; DROP TABLE IF EXISTS metric;
        CREATE TABLE obs(id INTEGER PRIMARY KEY, kind TEXT, vehicle INTEGER,
                         segment INTEGER, t REAL, received REAL, payload TEXT);
        CREATE TABLE metric(kind TEXT, identity TEXT, segment INTEGER, t REAL, value REAL);
    ''')
    count = rejected = 0
    with Path(journal).open('rb') as stream:
        while max_bytes is None or stream.tell() < max_bytes:
            line = stream.readline()
            if not line:
                break
            if max_bytes is not None and stream.tell() > max_bytes:
                break
            try:
                row = json.loads(line)
                if not isinstance(row, dict) or not finite(row.get('t')) or not finite(row.get('received_t')):
                    raise ValueError('Invalid timestamps')
                segment = int(row.get('segment', 0))
                vehicle = int(row.get('vehicle_id', 0))
                connection.execute('INSERT INTO obs VALUES(NULL,?,?,?,?,?,?)', (
                    str(row.get('kind', 'unknown')), vehicle, segment,
                    row['t'], row['received_t'], json.dumps(clean(row), allow_nan=False)))
                count += 1
            except (ValueError, TypeError, OverflowError):
                rejected += 1
            if (count+rejected) % 2000 == 0:
                connection.commit()
    connection.executescript('''
        CREATE INDEX obs_lookup ON obs(kind,vehicle,segment,t,id);
        CREATE INDEX metric_lookup ON metric(kind,identity,value);
    ''')
    connection.commit()
    return connection, count, rejected


def rows(db, kind, vehicle=None, window=None):
    query = 'SELECT payload FROM obs WHERE kind=?'
    args = [kind]
    if vehicle is not None:
        query += ' AND vehicle=?'
        args.append(vehicle)
    clause, bounds = window_clause(window)
    query += clause
    args += bounds
    for (payload,) in db.execute(query+' ORDER BY segment,t,id', args):
        yield json.loads(payload)


def latest(db, kind, vehicle, segment, stamp, max_age=None):
    found = db.execute('''SELECT t,payload FROM obs
        WHERE kind=? AND vehicle=? AND segment=? AND t<=?
        ORDER BY t DESC,id DESC LIMIT 1''', (kind,vehicle,segment,stamp)).fetchone()
    if found is None or (max_age is not None and stamp-found[0] > max_age):
        return None
    return json.loads(found[1])


def interpolate(db, kind, vehicle, segment, stamp, max_gap=0.25):
    before = latest(db,kind,vehicle,segment,stamp)
    if before is None or position(before) is None or (kind == 'estimate' and before.get('status') not in (1,2)):
        return None
    if abs(before['t']-stamp) < 1e-9:
        return position(before)
    after = db.execute('''SELECT payload FROM obs WHERE kind=? AND vehicle=?
        AND segment=? AND t>? ORDER BY t,id DESC LIMIT 1''',
        (kind,vehicle,segment,stamp)).fetchone()
    if after is None:
        return None
    after = json.loads(after[0])
    gap = after['t']-before['t']
    if (not 0.0 < gap <= max_gap or position(after) is None
            or (kind == 'estimate' and after.get('status') not in (1,2))):
        return None
    weight = (stamp-before['t'])/gap
    return tuple(a+weight*(b-a) for a,b in zip(position(before),position(after)))


def add(db, kind, identity, segment, stamp, value):
    if finite(value):
        db.execute('INSERT INTO metric VALUES(?,?,?,?,?)', (kind,str(identity),segment,stamp,value))


def summary(db, kind, identity):
    args = (kind,str(identity))
    n, mean, maximum, minimum, squares = db.execute('''SELECT COUNT(*),AVG(value),MAX(value),MIN(value),SUM(value*value)
        FROM metric WHERE kind=? AND identity=?''', args).fetchone()
    p95 = None
    if n:
        index = (n-1)*0.95
        lower = int(index)
        values = db.execute('''SELECT value FROM metric WHERE kind=? AND identity=?
            ORDER BY value LIMIT 2 OFFSET ?''', args+(lower,)).fetchall()
        p95 = values[0][0] if len(values) == 1 else values[0][0]+(index-lower)*(values[1][0]-values[0][0])
    return {'count':n, 'mean':mean, 'max':maximum, 'min':minimum,
            'rmse':math.sqrt(squares/n) if n else None, 'p95':p95}


def observed_rate(db, kind, vehicle, window=None):
    count = 0
    duration = 0.0
    clause, bounds = window_clause(window)
    for n, first, last in db.execute('''SELECT COUNT(DISTINCT t),MIN(t),MAX(t)
        FROM obs WHERE kind=? AND vehicle=?''' + clause + ' GROUP BY segment',
        [kind, vehicle] + bounds):
        if n > 1 and last > first:
            count += n-1
            duration += last-first
    return count/duration if duration else None


def active_state(db, vehicle, segment, stamp, external=None):
    state = latest(db,'state',vehicle,segment,stamp,0.3)
    if not state or not state.get('status_valid') or not state.get('armed') or state.get('failsafe'):
        return False
    if not finite(state.get('status_age_s')) or not 0 <= state['status_age_s'] <= 2.0:
        return False
    return external is None or (state.get('nav_state') == external and state.get('nav_state_user_intention') == external)


def goal_fields(status):
    data = status.get('data', {}) if status else {}
    goal = data.get('goal', [data.get('goal_east_m'), data.get('goal_north_m')])
    if isinstance(goal, dict):
        goal = [goal.get('east_m'), goal.get('north_m')]
    altitude = data.get('altitude_amsl_m', data.get('working_altitude_amsl_m'))
    return data, goal, altitude


def export_report(run_directory, output_directory=None, max_bytes=None, plots=False, window=None):
    window = measurement_window(window)
    run = Path(run_directory)
    journal = run/'observations.jsonl'
    available_bytes = journal.stat().st_size
    if max_bytes is None:
        max_bytes = available_bytes
    if (not isinstance(max_bytes,int) or isinstance(max_bytes,bool)
            or not 0 <= max_bytes <= available_bytes):
        raise ValueError('Captured journal prefix is invalid or no longer available')
    if output_directory is not None:
        output = Path(output_directory)
    elif window is None:
        output = run/'evaluation'
    else:
        output = run/'evaluation_windows'/(
            f"{window['segment']}_{window['start_sim_s']:.12g}_{window['end_sim_s']:.12g}")
    output.mkdir(parents=True,exist_ok=True)
    metadata = json.loads((run/'metadata.json').read_text())
    db, count, rejected = create_database(journal,output/'metrics.sqlite',max_bytes)
    ids = list(metadata['vehicle_ids'])
    leader = metadata.get('leader_id',ids[0])
    gap = float(metadata.get('truth_max_gap_s',0.25))
    danger = float(metadata.get('danger_distance_m',20.0))
    origin = float(metadata.get('origin_alt_amsl_m',0.0))
    external = int(metadata.get('external_nav_state',23))
    clause, bounds = window_clause(window)
    selected_count = db.execute('SELECT COUNT(*) FROM obs WHERE 1=1' + clause, bounds).fetchone()[0]
    coverage = {}
    for kind, vehicle, n, first, last in db.execute('''
            SELECT kind,vehicle,COUNT(*),MIN(t),MAX(t) FROM obs WHERE 1=1'''
            + clause + ' GROUP BY kind,vehicle', bounds):
        coverage.setdefault(kind, {})[str(vehicle)] = {
            'count': n, 'first_sim_s': first, 'last_sim_s': last}
    report = {'schema_version':1,'provenance':'recorded_ros_topics','observations':selected_count,
              'context_observations':count, 'journal_prefix_bytes':max_bytes,
              'measurement_window':window, 'coverage':coverage,
              'malformed_records':rejected,'metadata':metadata,'vehicles':{},'pairs':{},'missions':{},
              'definitions':{
                  'window':'Inclusive source-time bounds in one clock segment. Earlier/later rows in the captured journal prefix are context only. Malformed-record count covers the captured prefix.',
                  'truth_alignment':'Linear interpolation between bracketing GT samples; no extrapolation; same clock segment.',
                  'formation':'Actual GT relative offset minus target relative to time-aligned leader mesh; only confirmed External samples.',
                  'airborne_pair':'Both fresh armed states and GT ENU z above airborne_min_height_m; no landed field exists in VehicleState.',
                  'cross_track':'Horizontal distance to finite route polyline during cruise only.',
                  'percentile':'Exact interpolated rank percentile from disk-backed metric values.',
                  'loss':'Counter drops/generated; pending packets are not losses. Sequence gaps have a separately labelled observed-window scope.',
              }}
    for i in ids:
        unmatched = invalid = 0
        for row in rows(db,'estimate',i,window):
            segment,stamp = row.get('segment',0),row['t']
            age = row.get('measurement_age_s')
            if finite(age) and age >= 0:
                add(db,'measurement_age_s',i,segment,stamp,age)
            delay = row['received_t']-stamp
            if delay >= 0:
                add(db,'estimate_delivery_delay_s',i,segment,stamp,delay)
            if row.get('status') not in (1,2) or position(row) is None:
                invalid += 1
                continue
            actual = interpolate(db,'truth',i,segment,stamp,gap)
            if actual is None:
                unmatched += 1
                continue
            p = position(row)
            add(db,'localization_3d_m',i,segment,stamp,distance(p,actual))
            add(db,'localization_horizontal_m',i,segment,stamp,math.hypot(p[0]-actual[0],p[1]-actual[1]))
            add(db,'localization_vertical_m',i,segment,stamp,abs(p[2]-actual[2]))
        missing_targets = 0
        for row in rows(db,'target',i,window):
            segment,stamp = row.get('segment',0),row['t']
            if not active_state(db,i,segment,stamp,external) or position(row) is None:
                continue
            actual = interpolate(db,'truth',i,segment,stamp,gap)
            actual_leader = interpolate(db,'truth',leader,segment,stamp,gap)
            mesh_leader = interpolate(db,'estimate',leader,segment,stamp,gap)
            if actual is None:
                missing_targets += 1
                continue
            add(db,'target_tracking_m',i,segment,stamp,distance(actual,position(row)))
            if actual_leader is not None and mesh_leader is not None:
                actual_offset = tuple(a-b for a,b in zip(actual,actual_leader))
                desired_offset = tuple(a-b for a,b in zip(position(row),mesh_leader))
                add(db,'formation_relative_m',i,segment,stamp,distance(actual_offset,desired_offset))
        report['vehicles'][str(i)] = {
            name:summary(db,name,i) for name in ('localization_3d_m','localization_horizontal_m',
                'localization_vertical_m','formation_relative_m','target_tracking_m',
                'measurement_age_s','estimate_delivery_delay_s')}
        report['vehicles'][str(i)].update({'unmatched_truth':unmatched,'invalid_estimates':invalid,
            'unmatched_targets':missing_targets,'position_update_rate_hz':observed_rate(db,'estimate',i,window)})
    for first,second in itertools.combinations(ids,2):
        pair = f'{first}-{second}'
        episodes = all_episodes = 0
        previous = previous_air = None
        for row in rows(db,'truth',first,window):
            segment,stamp = row.get('segment',0),row['t']
            a = position(row)
            b = interpolate(db,'truth',second,segment,stamp,gap)
            if a is None or b is None:
                previous = previous_air = None
                continue
            d = distance(a,b)
            add(db,'separation_all_m',pair,segment,stamp,d)
            close = d < danger
            continuous = previous is not None and previous[0] == segment and 0 <= stamp-previous[1] <= gap
            if close and (not continuous or not previous[2]):
                all_episodes += 1
            previous = (segment,stamp,close)
            airborne = (min(a[2],b[2]) > metadata.get('airborne_min_height_m',10.0)
                        and active_state(db,first,segment,stamp) and active_state(db,second,segment,stamp))
            if not airborne:
                previous_air = None
                continue
            add(db,'separation_airborne_m',pair,segment,stamp,d)
            continuous = previous_air is not None and previous_air[0] == segment and 0 <= stamp-previous_air[1] <= gap
            if close and (not continuous or not previous_air[2]):
                episodes += 1
            previous_air = (segment,stamp,close)
        report['pairs'][pair] = {'all_samples':summary(db,'separation_all_m',pair),
            'airborne_samples':summary(db,'separation_airborne_m',pair),
            'danger_threshold_m':danger,
            'all_observed_close_episodes':all_episodes if summary(db,'separation_all_m',pair)['count'] else None,
            'airborne_observed_close_episodes':episodes if summary(db,'separation_airborne_m',pair)['count'] else None,
            'episode_scope':'Observation starts at the selected window boundary. A gap over truth_max_gap_s ends continuity; a later close sample starts a new observed episode.'}
    mission_context = []
    if window is not None:
        previous = latest(db,'mission',0,window['segment'],window['start_sim_s'])
        if previous is not None:
            mission_context.append(previous)
    for row in itertools.chain(mission_context, rows(db,'mission',window=window)):
        data,goal,altitude = goal_fields(row)
        identity = str(row.get('segment',0))+':'+str(data.get('mission_id','unknown'))
        mission = report['missions'].setdefault(identity,{'first_observed_sim_s':row['t'],
            'start_sim_s':data.get('start_sim_s',row['t']),'reported_time_to_goal_s':None,
            'gt_time_to_goal_s':None,'last_phase':None})
        mission['last_phase'] = data.get('phase')
        if data.get('reached') and mission['reported_time_to_goal_s'] is None:
            reached = data.get('reached_sim_s')
            if window is None or (finite(reached) and in_window(row.get('segment',0),reached,window)):
                mission['reported_time_to_goal_s'] = (reached-mission['start_sim_s']
                    if finite(reached) and finite(mission['start_sim_s']) else data.get('elapsed_s'))
    first_gt_reach = {}
    # Earlier truth remains context for determining whether the *first* goal
    # encounter occurred before the window. It never adds geometric error samples.
    for row in rows(db,'truth',leader):
        segment,stamp = row.get('segment',0),row['t']
        if window is not None and (segment != window['segment'] or stamp > window['end_sim_s']):
            continue
        actual = position(row)
        status = latest(db,'mission',0,segment,stamp,2.0)
        data,goal,altitude = goal_fields(status)
        if actual is None or data.get('phase') != 'cruise':
            continue
        identity = str(segment)+':'+str(data.get('mission_id','unknown'))
        if in_window(segment,stamp,window):
            route = latest(db,'route',0,segment,stamp)
            points = route.get('points',[]) if route else []
            add(db,'cross_track_m',identity,segment,stamp,cross_track(actual,points))
            if finite(altitude):
                add(db,'route_altitude_error_m',identity,segment,stamp,abs(origin+actual[2]-altitude))
        mission = report['missions'].get(identity)
        radius = data.get('arrival_radius_m',metadata.get('goal_radius_m',30.0))
        if (mission and identity not in first_gt_reach and isinstance(goal,list)
                and len(goal)>=2 and all(finite(v) for v in goal[:2]) and finite(altitude)
                and math.hypot(actual[0]-goal[0],actual[1]-goal[1]) <= radius
                and abs(origin+actual[2]-altitude) <= metadata.get('goal_height_tolerance_m',10.0)):
            first_gt_reach[identity] = stamp
            if in_window(segment,stamp,window) and finite(mission['start_sim_s']):
                mission['gt_time_to_goal_s'] = stamp-mission['start_sim_s']
    for identity,mission in report['missions'].items():
        mission['cross_track_m'] = summary(db,'cross_track_m',identity)
        mission['altitude_error_m'] = summary(db,'route_altitude_error_m',identity)
    report['links'] = link_metrics(db,window)
    report['definitions']['mission_time'] = (
        'Time from mission start, not measurement start; only a goal event occurring '
        'inside the selected interval is timed. GT uses its first observed goal encounter '
        'during cruise, including earlier journal context. No observation is not proof of absence.')
    db.commit()
    kinds = [r[0] for r in db.execute('SELECT DISTINCT kind FROM metric')]
    for kind in kinds:
        with (output/(kind+'.csv')).open('w',newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(('identity','segment','sim_time_s','value'))
            writer.writerows(db.execute('SELECT identity,segment,t,value FROM metric WHERE kind=? ORDER BY identity,segment,t',(kind,)))
    (output/'metrics.json').write_text(json.dumps(clean(report),indent=2,allow_nan=False)+'\n')
    if window is not None:
        (output/'window.json').write_text(json.dumps(
            {**window, 'max_bytes': max_bytes},indent=2,allow_nan=False)+'\n')
    if plots:
        plot_metrics(db,output)
    db.close()
    return clean(report)


def counter_metrics(db, kind, window):
    """Difference only observed snapshots; never assign old cumulative losses to a window."""
    previous, sessions, endpoints = {}, {}, {}
    counters = ('generated', 'dropped', 'delivered')
    for row in rows(db,kind,window=window):
        data = row.get('data',{})
        links = data.get('links',[])
        if not isinstance(links,list):
            continue
        for link in links:
            if not isinstance(link,dict):
                continue
            if not all(isinstance(link.get(k),int) and not isinstance(link[k],bool)
                       and link[k] >= 0 for k in counters):
                continue
            identity = f"{link.get('vehicle_id')}:{link.get('station_id',link.get('kind','unknown'))}"
            base = (row.get('segment',0),data.get('epoch',0),identity)
            prior = previous.get(base)
            monotonic = counters + ('delay_sum_s', 'delay_count')
            if prior is not None and any(
                    finite(link.get(k)) and finite(prior.get(k)) and link[k] < prior[k]
                    for k in monotonic):
                sessions[base] = sessions.get(base,0)+1
            previous[base] = link
            name = ':'.join(map(str,base))+f":session{sessions.get(base,0)}"
            entry = endpoints.setdefault(name, {
                'first':link, 'first_t':row['t'], 'count':0})
            entry.update(last=link, last_t=row['t'], count=entry['count']+1)
    result = {}
    for name, entry in endpoints.items():
        first, last = entry['first'], entry['last']
        complete = window is None or (entry['count'] >= 2 and entry['last_t'] > entry['first_t'])
        values = {key:(last[key] if window is None else last[key]-first[key])
                  if complete else None for key in counters}
        generated, dropped = values['generated'], values['dropped']
        delay_sum, delay_n = last.get('delay_sum_s'), last.get('delay_count')
        if window is not None:
            delay_sum = delay_sum-first['delay_sum_s'] if (
                complete and finite(delay_sum) and finite(first.get('delay_sum_s'))) else None
            delay_n = delay_n-first['delay_count'] if (
                complete and finite(delay_n) and finite(first.get('delay_count'))) else None
        values.update(
            pending=last.get('pending'),
            loss_fraction=dropped/generated if generated else None,
            mean_delivery_delay_s=delay_sum/delay_n if (
                finite(delay_sum) and delay_sum >= 0 and finite(delay_n) and delay_n > 0) else None,
            coverage={'first_snapshot_sim_s':entry['first_t'], 'last_snapshot_sim_s':entry['last_t'],
                      'duration_sim_s':entry['last_t']-entry['first_t'],
                      'snapshot_count':entry['count'], 'sufficient_snapshots':complete},
            scope=('Simulator cumulative counters in one counter epoch/session; pending is not a loss'
                   if window is None else
                   'Counter differences between first and last snapshots INSIDE the measurement window. '
                   'Window edges outside snapshot coverage are unknown. Pending is the last snapshot '
                   'queue size, not a loss. Delivered packets may have been queued before the window.'))
        result[name] = values
    return result


def link_metrics(db, window=None):
    result = {'ranging_counter_sessions':counter_metrics(db,'ranging_stats',window),
              'network_counter_sessions':counter_metrics(db,'network_stats',window),
              'ranging_observed_windows':{}}
    # Every packet's sequence is incremented before loss simulation.
    aggregates = {}
    for row in rows(db,'range',window=window):
        sequence = row.get('sequence')
        if not isinstance(sequence,int) or sequence<0:
            continue
        key = (row.get('segment',0),row['vehicle_id'],row.get('station_id'))
        delay = row['received_t']-row['t']
        if delay >= 0:
            add(db,'range_delivery_delay_s',':'.join(map(str,key)),key[0],row['t'],delay)
        a = aggregates.setdefault(key,{'min':sequence,'max':sequence,'received':0})
        a['min'],a['max'],a['received'] = min(a['min'],sequence),max(a['max'],sequence),a['received']+1
    for key,a in aggregates.items():
        # DISTINCT is evaluated on disk; duplicates do not hide missing sequences.
        clause, bounds = window_clause(window)
        unique = db.execute('''SELECT COUNT(DISTINCT json_extract(payload,'$.sequence')) FROM obs
            WHERE kind='range' AND segment=? AND vehicle=? AND json_extract(payload,'$.station_id')=?'''
            + clause,list(key)+bounds).fetchone()[0]
        span = a['max']-a['min']+1
        result['ranging_observed_windows'][':'.join(map(str,key))] = {
            'received':a['received'],'unique_received':unique,'duplicates':a['received']-unique,
            'delivery_delay_s':summary(db,'range_delivery_delay_s',':'.join(map(str,key))),
            'observed_sequence_span':span,'missing_within_window':span-unique,
            'loss_fraction_within_window':(span-unique)/span,
            'scope':'First to last received sequence only; leading/trailing losses are unknown'}
    return result


def plot_metrics(db, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    kinds = ('localization_3d_m','formation_relative_m','separation_airborne_m','cross_track_m','route_altitude_error_m')
    for kind in kinds:
        groups = list(db.execute('SELECT DISTINCT identity,segment FROM metric WHERE kind=?',(kind,)))
        if not groups:
            continue
        fig,axis = plt.subplots(figsize=(10,4),layout='constrained')
        for identity,segment in groups:
            n = db.execute('SELECT COUNT(*) FROM metric WHERE kind=? AND identity=? AND segment=?',(kind,identity,segment)).fetchone()[0]
            stride = max(1,math.ceil(n/10000))
            samples = [r for index,r in enumerate(db.execute('SELECT t,value FROM metric WHERE kind=? AND identity=? AND segment=? ORDER BY t',(kind,identity,segment))) if index%stride == 0]
            axis.plot([p[0] for p in samples],[p[1] for p in samples],label=f'{identity}, epoch {segment}',linewidth=1)
        axis.set(xlabel='Simulation time [s]',ylabel=kind,title='Recorded measurements')
        axis.grid(alpha=.3)
        axis.legend()
        fig.savefig(output/(kind+'.png'),dpi=150)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory',type=Path)
    parser.add_argument('--output-directory',type=Path)
    parser.add_argument('--plots',action='store_true')
    parser.add_argument('--window-file',type=Path,
                        help='JSON with segment/start_sim_s/end_sim_s and optional frozen max_bytes')
    args = parser.parse_args()
    window = None
    max_bytes = None
    if args.window_file is not None:
        window = json.loads(args.window_file.read_text())
        measurement_window(window)
        max_bytes = window.get('max_bytes')
        if max_bytes is not None and (not isinstance(max_bytes,int)
                or isinstance(max_bytes,bool) or max_bytes <= 0):
            parser.error('window max_bytes must be a positive integer')
    report = export_report(args.run_directory,args.output_directory,max_bytes=max_bytes,
                           plots=args.plots,window=window)
    print(json.dumps({'observations':report['observations'],'vehicles':list(report['vehicles'])}))


if __name__ == '__main__':
    main()
