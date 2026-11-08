"""Headless proof that the cursor does not reset across refresh ticks.

The bug: _render_* called table.clear() every refresh, which set cursor_row
back to 0 once a second. This drives the real app, moves the cursor to a
non-zero row, runs many refresh cycles, and asserts it stayed put.
"""
import asyncio
import os
import sys

os.environ.setdefault("TERM", "xterm-256color")

from jupiter_monitor_tui.panel import JupiterPanel


async def main():
    app = JupiterPanel(interval=0.15)      # fast ticks to make the bug obvious
    async with app.run_test() as pilot:
        await pilot.pause()

        failures = []
        for tab_id, table_id in (("overview", "#t_overview"),
                                 ("process", "#t_process"),
                                 ("arm", "#t_arm")):
            app.query_one("TabbedContent").active = tab_id
            await pilot.pause()
            await pilot.pause()
            table = app.query_one(table_id)

            if table.row_count < 3:
                print("%-10s SKIP (only %d rows)" % (tab_id, table.row_count))
                continue

            # Focus the table so arrow keys reach it, then move down 3 rows.
            table.focus()
            await pilot.pause()
            for _ in range(3):
                await pilot.press("down")
                await pilot.pause()
            start = table.cursor_row
            if start == 0:
                print("%-10s SKIP (could not move cursor)" % tab_id)
                continue

            # Let several refresh intervals pass.
            for _ in range(10):
                await pilot.pause(0.2)

            end = table.cursor_row
            ok = (end == start)
            print("%-10s start_row=%-3s after_10_ticks=%-3s  %s"
                  % (tab_id, start, end, "OK" if ok else "REGRESSED"))
            if not ok:
                failures.append(tab_id)

        arm = app.query_one("#t_arm")
        print("arm row_count =", arm.row_count)

        if failures:
            print("FAILED:", ", ".join(failures))
            raise SystemExit(1)

    print("CURSOR_STABLE_OK")


asyncio.run(main())
