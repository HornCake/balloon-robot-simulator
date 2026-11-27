"""Startup selector for terminals with curses support."""
import argparse
import shlex
import sys
import unicodedata


def fitting_prefix(text, width):
    used = 0
    for index, char in enumerate(text):
        used += 2 if unicodedata.east_asian_width(char) in ('W', 'F') else 1
        if used > width:
            return text[:index]
    return text


def draw_line(screen, row, text, style=0, x=0, limit=None):
    height, width = screen.getmaxyx()
    if row >= height:
        return
    width -= x + (row == height - 1)  # Avoid writing to the terminal's bottom-right cell.
    if limit is not None:
        width = min(width, limit)
    line = fitting_prefix(text, width)
    if line:
        screen.addstr(row, x, line, style)


def description_lines(text, width, rows):
    for row in range(rows):
        line = fitting_prefix(text, width)
        if line == text or not line:
            if line:
                yield line
            return
        if row == rows - 1:
            yield fitting_prefix(line, width - 3).rstrip() + '...'
            return
        space = line.rfind(' ')
        if space > len(line) // 2 and text[len(line)] != ' ':
            line = line[:space]
        yield line
        text = text[len(line):].lstrip()


def select_startup(args, parser, catalog, terminal):
    commands = {'help', 'interactive', 'list_patterns'}
    fields = {action.dest: action for action in parser._actions if action.dest not in commands}
    # Each selector edits one mutually exclusive family of CLI options.
    selectors = {
        '開始方法': ('すぐ開始', {'start_paused': '停止', 'start_delay': '秒数指定'}),
        '重りの配分': ('設定に従う', {'tilt_deg': '傾き', 'ballast_ratios': '配分比'}),
        '関節の動き': ('設定に従う', {'mode': '制御方式', 'pattern': 'パターン'}),
        '比較対象': ('単独', {'compare': '空力・設定内ケース', 'compare_config': '設定ファイル',
                           'compare_patterns': 'パターン'}),
    }
    groups = [
        ('実行', ['config', 'duration', 'headless', 'realtime', '開始方法']),
        ('物理・流体', ['neutral', '重りの配分', 'no_buoyancy', 'no_fluid', 'fluid_model']),
        ('動作', ['関節の動き', 'motion_scale', 'period', 'phase_step_deg']),
        ('比較', ['比較対象', 'focus_case', 'side_all']),
        ('カメラ・表示', ['camera', 'projection', 'x_direction', 'windowed']),
        ('出力', ['output', 'plot', 'export_model']),
        ('形状・メッシュ', ['dimensions', 'mesh', 'mesh_scale', 'mesh_rotation', 'mesh_center', 'fit_mesh']),
    ]
    known = {item for _, members in groups for item in members}
    known.update(field for _, members in selectors.values() for field in members)
    extra = [item for item in fields if item not in known]
    if extra:
        groups.append(('その他', extra))
    groups.append(('動作パターン一覧', list(catalog)))
    items = [item for _, members in groups for item in members]
    error = ''

    def choices(action):
        if isinstance(action, argparse.BooleanOptionalAction):
            return [None, True, False]
        if action.choices and action.nargs is None:
            return list(dict.fromkeys([action.default, *action.choices]))
        return None

    def labels():
        result = []
        for item in items:
            if item in selectors:
                default, members = selectors[item]
                label = default
                for field, name in members.items():
                    value = getattr(args, field)
                    if value != fields[field].default:
                        detail = str(value)
                        if isinstance(value, list):
                            detail = shlex.join(map(str, value)) or '自動'
                        label = name if value is True else f'{name} {detail}'
                        break
                label = f'{item}　{label}'
            elif item in fields:
                action = fields[item]
                value = getattr(args, item)
                flag = action.option_strings[0]
                if action.nargs == 0 and choices(action) is None:
                    label = ('[x] ' if value else '[ ] ') + flag
                else:
                    if isinstance(value, list):
                        value = shlex.join([str(v) for v in value]) or "自動"
                    label = f'{flag}　{value if value is not None else "既定"}'
            else:
                selected = args.compare_patterns or [args.pattern]
                label = ('[x]' if item in selected else '[ ]') + f' {item}　{catalog[item]["name"]}'
            result.append(label)
        return result

    def description(index):
        item = items[index]
        if item in selectors:
            _, members = selectors[item]
            return ' / '.join(f'{fields[field].option_strings[0]} {fields[field].help}' for field in members)
        if item in fields:
            action = fields[item]
            return f'{action.option_strings[0]}  {action.help or "説明はありません。"}'
        pattern = catalog[item]
        return f'{item}  {pattern.get("description") or pattern["name"]}'

    def toggle(index, read_value):
        nonlocal error
        error = ''
        item = items[index]
        grouped = item in selectors
        if grouped:
            default, members = selectors[item]
            names = [default, *members.values()]
            hint = ' '.join(f'{i}={name}' for i, name in enumerate(names))
            selection = read_value(hint)
            if not selection:
                return
            if selection not in map(str, range(len(names))):
                error = '表示された番号を入力してください'
                return
            if selection == '0':
                for field in members:
                    setattr(args, field, fields[field].default)
                return
            item = list(members)[int(selection) - 1]
        if item in fields:
            action = fields[item]
            current = getattr(args, item)
            options = choices(action)
            if options and not grouped:
                value = options[(options.index(current) + 1) % len(options)]
            elif action.nargs == 0:
                value = True if grouped else not current
            else:
                hint = '（[]で自動、空欄で解除）' if action.nargs == '*' else ''
                if item == 'compare':
                    hint = '（空欄で自動 / ellipsoid / mesh_surface）'
                elif grouped and action.choices:
                    hint = '（' + ' '.join(action.choices) + '）' if item == 'mode' else '（候補は一覧参照）'
                raw = read_value(action.option_strings[0] + hint)
                if raw is None:
                    return
                try:
                    if not raw:
                        value = [] if grouped and action.nargs == '*' else action.default
                    elif action.nargs is None:
                        if action.choices and raw not in action.choices:
                            raise ValueError('無効な選択肢: ' + raw)
                        value = (action.type or str)(raw)
                    else:
                        tokens = shlex.split(raw)
                        if action.nargs == '*' and raw.strip() == '[]':
                            tokens = []
                        if action.choices and any(v not in action.choices for v in tokens):
                            raise ValueError('無効な選択肢: ' + raw)
                        value = [(action.type or str)(v) for v in tokens]
                        if (not value and action.nargs != '*') or (isinstance(action.nargs, int) and len(value) != action.nargs):
                            raise ValueError('値の個数を確認してください')
                except (ValueError, argparse.ArgumentTypeError) as exc:
                    error = str(exc)
                    return
            if value is not None:
                for group in parser._mutually_exclusive_groups:
                    if action in group._group_actions and (grouped or value != action.default):
                        for peer in group._group_actions:
                            if peer is not action:
                                setattr(args, peer.dest, peer.default)
                if item in ('pattern', 'mode'):
                    args.compare_patterns = None
                elif item == 'compare_patterns':
                    args.mode = args.pattern = None
            setattr(args, item, value)
            return

        selected = list(args.compare_patterns or ([args.pattern] if args.pattern else []))
        if item in selected:
            selected.remove(item)
        else:
            selected.append(item)
        args.mode = args.compare = args.compare_config = None
        args.pattern = selected[0] if len(selected) == 1 else None
        args.compare_patterns = selected if len(selected) > 1 else None

    def render(screen):
        try:
            curses.curs_set(0)
        except curses.error:
            pass
        # ncurses enables click tracking via terminfo. xterm-compatible terminals
        # need mode 1003 to also send motion without a pressed button.
        click = curses.BUTTON1_CLICKED | curses.BUTTON1_PRESSED
        mask, old_mask = curses.mousemask(click | curses.REPORT_MOUSE_POSITION)
        track_motion = (mask & curses.REPORT_MOUSE_POSITION
                        and curses.tigetstr('kmous') in (b'\x1b[M', b'\x1b[<'))
        try:
            if track_motion:
                print('\x1b[?1003h', end='', flush=True)
            render_menu(screen, click & mask)
        finally:
            if track_motion:
                print('\x1b[?1003l', end='', flush=True)
            curses.mousemask(old_mask)

    def render_menu(screen, click):
        def read_value(flag):
            value = ''
            while True:
                row = screen.getmaxyx()[0] - 1
                screen.move(row, 0)
                screen.clrtoeol()
                draw_line(screen, row, f'{flag}（Enter 確定 / Esc 取消） {value}▏')
                key = screen.get_wch()
                if key in ('\n', '\r', curses.KEY_ENTER):
                    return value
                if key == '\x1b':
                    return None
                if key in ('\b', '\x7f', curses.KEY_BACKSPACE):
                    value = value[:-1]
                elif isinstance(key, str) and key.isprintable():
                    value += key

        cursor = top = 0
        edit_on_redraw = False
        while True:
            height, width = screen.getmaxyx()
            columns = 2 if width >= 80 else 1
            cell = width // columns
            gutter = 1 if columns == 2 else 0
            headings = []
            positions = []
            y = 0
            for start in range(0, len(groups), columns):
                band = groups[start:start + columns]
                for col, (title, members) in enumerate(band):
                    headings.append((y, col, title))
                    positions.extend((y + offset, col) for offset in range(1, len(members) + 1))
                y += max(len(members) for _, members in band) + 2
            current_y, current_col = positions[cursor]
            help_rows = min(4, height // 4) if height >= 6 else 0
            rows = max(1, height - 2 - (help_rows + 1 if help_rows else 0))
            footer = rows + 1
            top = max(current_y - rows + 1, min(top, current_y))
            screen.erase()
            section = next(title for title, members in groups if items[cursor] in members)
            draw_line(screen, 0, '起動設定　/　' + section, curses.A_BOLD)
            if gutter:
                for row in range(1, footer):
                    draw_line(screen, row, '│', curses.A_DIM, cell - 1, 1)
            for row, col, title in headings:
                if top <= row < top + rows:
                    draw_line(screen, row - top + 1, '【' + title + '】', curses.A_BOLD,
                              col * cell, cell - gutter)
            visible_items = {}
            for i, ((row, col), label) in enumerate(zip(positions, labels())):
                if top <= row < top + rows:
                    screen_row = row - top + 1
                    visible_items[screen_row, col] = i
                    draw_line(screen, screen_row, label, curses.A_REVERSE if i == cursor else 0,
                              col * cell, cell - gutter)
            if help_rows:
                divider = '─' * width
                if gutter:
                    divider = divider[:cell - 1] + '┴' + divider[cell:]
                draw_line(screen, footer, divider, curses.A_DIM)
                for offset, line in enumerate(description_lines(description(cursor), width, help_rows), 1):
                    draw_line(screen, footer + offset, line)
            edit = 'Space/クリック' if click else 'Space'
            controls = f'Enter 実行  q 終了  {edit} 変更  矢印/hjkl 移動  Tab 次の分類'
            draw_line(screen, height - 1, error or controls)
            screen.refresh()
            if edit_on_redraw:
                edit_on_redraw = False
                toggle(cursor, read_value)
                continue
            key = screen.getch()
            if key in (10, 13, curses.KEY_ENTER):
                return
            if key in (27, ord('q')):
                raise SystemExit(0)
            if key == curses.KEY_MOUSE:
                try:
                    _, x, y, _, state = curses.getmouse()
                except curses.error:
                    continue
                item = visible_items.get((y, x // cell))
                if item is not None and x % cell < cell - gutter:
                    if state & (click | curses.REPORT_MOUSE_POSITION):
                        cursor = item
                    edit_on_redraw = bool(state & click)
            elif key in (ord('j'), curses.KEY_DOWN, ord('k'), curses.KEY_UP):
                step = 1 if key in (ord('j'), curses.KEY_DOWN) else -1
                candidates = [(abs(row - current_y), i) for i, (row, col) in enumerate(positions)
                              if col == current_col and (row - current_y) * step > 0]
                if candidates:
                    cursor = min(candidates)[1]
            elif key in (ord('h'), curses.KEY_LEFT, ord('l'), curses.KEY_RIGHT):
                target = current_col + (1 if key in (ord('l'), curses.KEY_RIGHT) else -1)
                candidates = [(abs(row - current_y), i) for i, (row, col) in enumerate(positions)
                              if col == target]
                if candidates:
                    cursor = min(candidates)[1]
            elif key == 9:
                starts = [items.index(members[0]) for _, members in groups if members]
                cursor = next((i for i in starts if i > cursor), 0)
            elif key == ord(' ') and items:
                toggle(cursor, read_value)

    if terminal:
        try:
            import curses
        except ImportError:
            terminal = False
        else:
            try:
                curses.wrapper(render)
            except curses.error:
                terminal = False
    if not terminal:
        selected = None
        while True:
            entries = iter(enumerate(labels(), 1))
            for title, members in groups:
                print(f'【{title}】')
                for _ in members:
                    i, label = next(entries)
                    print(f'{i}. {label}')
            if selected is not None:
                print('─' * 48)
                print(description(selected))
            if error:
                print(error)
            answer = input('番号で変更、Enterで実行、qで終了: ').strip()
            if not answer:
                break
            if answer == 'q':
                raise SystemExit(0)
            if answer in map(str, range(1, len(items) + 1)):
                selected = int(answer) - 1
                print('─' * 48)
                print(description(selected))
                toggle(selected, lambda flag: input(flag + ": "))
