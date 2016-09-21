import test_bakery
import bakery
import createBakery
import imp


imp.reload(test_bakery)
imp.reload(createBakery)
imp.reload(bakery)

test_bakery.main('Object')