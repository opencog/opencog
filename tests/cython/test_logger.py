from unittest import TestCase
import os
import tempfile
import time

import opencog.logger

class LoggerTest(TestCase):

    def setUp(self):
        tempfd, self.tempfn = tempfile.mkstemp()
        # close the temp file as Logger will want to manually
        # open it
        os.close(tempfd) 
        self.log = opencog.logger.create_logger(self.tempfn)

    def tearDown(self):
        del self.log
        os.remove(self.tempfn)

    def test_log_levels(self):
        self.log.set_level("ERROR")
        self.assertFiltersCorrectly(["ERROR"],["WARN", "INFO", "DEBUG", "FINE"])
        self.log.set_level("WARN")
        self.assertFiltersCorrectly(["ERROR","WARN"],["INFO", "DEBUG", "FINE"])
        self.log.set_level("INFO")
        self.assertFiltersCorrectly(["ERROR","WARN","INFO"],["DEBUG", "FINE"])
        self.log.set_level("DEBUG")
        self.assertFiltersCorrectly(["ERROR","WARN","INFO","DEBUG"],["FINE"])
        self.log.set_level("FINE")
        self.assertFiltersCorrectly(["ERROR","WARN","INFO","DEBUG","FINE"],[])

    def assertFiltersCorrectly(self,lvls_displayed,lvls_muted):
        file_size = os.path.getsize(self.tempfn)
        for lvlname in lvls_muted:
            lvl = self.log.string_as_level(lvlname)
            self.log.log(lvl,"these messages should be muted")
            new_size = os.path.getsize(self.tempfn)
            self.assertEquals(file_size, new_size)
        for lvlname in lvls_displayed:
            lvl = self.log.string_as_level(lvlname)
            self.log.log(lvl,"this should appear")
            # sleep because messages are written by another loop
            time.sleep(0.1)
            new_size = os.path.getsize(self.tempfn)
            self.assertTrue(file_size < new_size)
            file_size = new_size

    def test_get_set_log_level(self):
        for lvlname in self.log.level_order():
            self.log.set_level(lvlname)
            self.assertEquals(self.log.get_level(),lvlname)


