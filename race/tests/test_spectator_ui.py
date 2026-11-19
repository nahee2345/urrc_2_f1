"""Spectator regression tests: free-first, all-team follow, no control API."""
from pathlib import Path
import unittest


HTML = Path(__file__).resolve().parents[1] / 'spectator/index.html'
SERVER = Path(__file__).resolve().parents[1] / 'scripts/spectator.py'


class SpectatorUITests(unittest.TestCase):
    def setUp(self):
        self.text = HTML.read_text(encoding='utf-8')
        self.js = self.text.split('<script>', 1)[1].split('</script>', 1)[0]

    def test_free_mode_is_initial_and_no_autofollow_on_connect(self):
        self.assertIn("mode='free'", self.js)
        self.assertIn('No auto-follow on initial connection', self.js)
        self.assertNotIn('chosen=data.leader||null', self.js)

    def test_mouse_pan_zoom_rotate_available(self):
        for action in ('pointerdown', 'pointermove', 'pointerup', 'wheel', 'contextmenu', 'screenToMap', 'mapToScreen', 'zoomAt'):
            self.assertIn(action, self.js)

    def test_full_track_and_independent_camera(self):
        self.assertIn("$('fit').addEventListener('click',fit)", self.js)
        self.assertIn('loadTrack(trackSelect.value)', self.js)
        self.assertIn('function drawFree', self.js)
        self.assertIn('function drawFollow', self.js)

    def test_any_running_car_can_be_followed(self):
        self.assertIn('function follow(team)', self.js)
        self.assertIn('for(const row of latest?.teams||[])', self.js)
        self.assertIn('if(team)follow(team)', self.js)
        self.assertIn('div.addEventListener(\'click\',()=>follow(r.team))', self.js)

    def test_dnf_falls_back_only_during_follow(self):
        self.assertIn("if(mode==='follow'&&!activeCar(selected))", self.js)
        self.assertIn('if(data.leader&&activeCar(data.leader))follow(data.leader)', self.js)
        self.assertIn('else toFree(false)', self.js)

    def test_no_network_control_commands(self):
        self.assertNotIn("method:'POST'", self.js)
        self.assertNotIn('ros2 topic pub', self.text)
        server = SERVER.read_text(encoding='utf-8')
        for name in ('do_POST', 'do_PUT', 'do_PATCH', 'do_DELETE'):
            self.assertIn(name+'(self):self.reply(405', server)


if __name__ == '__main__':
    unittest.main()
