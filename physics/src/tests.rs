#[cfg(test)]
#[test]
fn sandbox() {
    struct Test {
	pub something: f32,
    }
    impl Test {
	pub fn new(v: f32) -> Test{
	    Test {
		something: v,
	    }
	}
    }

    fn do_something<'a, I>(it: I)
    where I:Iterator<Item=&'a mut Test>, {
	it.for_each(|x| x.something = 1.0)
    }
    
    let mut mut_vec = vec![Test::new(1.0), Test::new(2.0)];
    mut_vec[0].something = 12.0;

    let subset = mut_vec.iter_mut().filter_map(|x| Some(x));
    do_something(subset);
    assert!(mut_vec[0].something == 1.0);
    assert!(mut_vec[1].something == 1.0);
}

